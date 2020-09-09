/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <vector>
#include <functional>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <ignition/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_request.hpp>

#include <rmf_plugins_common/ingestor_common.hpp>
#include <rmf_plugins_common/utils.hpp>

using namespace rmf_plugins_utils;
using namespace rmf_ingestor_common;

namespace rmf_gazebo_plugins {

class TeleportIngestorPlugin : public gazebo::ModelPlugin
{

public:

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using IngestorState = rmf_ingestor_msgs::msg::IngestorState;
  using IngestorRequest = rmf_ingestor_msgs::msg::IngestorRequest;
  using IngestorResult = rmf_ingestor_msgs::msg::IngestorResult;

  TeleportIngestorPlugin();
  ~TeleportIngestorPlugin();
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

private:
  // Stores params representing state of Ingestor, and handles all message pub/sub
  std::unique_ptr<TeleportIngestorCommon> _ingestor_common;

  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  gazebo::physics::ModelPtr _ingested_model; // Item that ingestor may contain
  gazebo::physics::WorldPtr _world;

  gazebo_ros::Node::SharedPtr _node;

  bool find_nearest_model(
    const std::vector<gazebo::physics::ModelPtr>& models,
    gazebo::physics::ModelPtr& nearest_model) const;
  bool get_payload_model(
    const gazebo::physics::ModelPtr& robot_model,
    gazebo::physics::ModelPtr& payload_model) const;
  bool ingest_from_nearest_robot(const std::string& fleet_name);
  void send_ingested_item_home();
  void on_update();
};

bool TeleportIngestorPlugin::find_nearest_model(
  const std::vector<gazebo::physics::ModelPtr>& models,
  gazebo::physics::ModelPtr& nearest_model) const
{
  double nearest_dist = 1e6;
  bool found = false;

  for (const auto& m : models)
  {
    if (!m || m->IsStatic() || m->GetName() == _model->GetName())
      continue;

    const double dist =
      m->WorldPose().Pos().Distance(_model->WorldPose().Pos());
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      nearest_model = m;
      found = true;
    }
  }
  return found;
}

bool TeleportIngestorPlugin::get_payload_model(
  const gazebo::physics::ModelPtr& robot_model,
  gazebo::physics::ModelPtr& payload_model) const
{
  if (!robot_model)
    return false;

  const auto robot_collision_bb = robot_model->BoundingBox();
  ignition::math::Vector3d max_corner = robot_collision_bb.Max();

  // Create a new bounding box extended slightly in the Z direction
  max_corner.Z(max_corner.Z() + 0.1);
  #if GAZEBO_MAJOR_VERSION <= 9
  const ignition::math::Box vicinity_box(
    robot_collision_bb.Min(), max_corner);
  #else
  const ignition::math::AxisAlignedBox vicinity_box(
    robot_collision_bb.Min(), max_corner);
  #endif

  // There might not be a better way to loop through all the models, as we
  // might consider delivering items that were spawned during run time,
  // instead of during launch.
  const auto robot_model_pos = robot_model->WorldPose().Pos();
  double nearest_dist = 1.0;
  const auto model_list = _world->Models();
  bool found = false;
  for (const auto& m : model_list)
  {
    if (!m ||
      m->IsStatic() ||
      m->GetName() == _model->GetName() ||
      m->GetName() == robot_model->GetName())
      continue;

    const double dist = m->WorldPose().Pos().Distance(robot_model_pos);
    if (dist < nearest_dist && vicinity_box.Intersects(m->BoundingBox()))
    {
      payload_model = m;
      nearest_dist = dist;
      found = true;
    }
  }
  return found;
}

bool TeleportIngestorPlugin::ingest_from_nearest_robot(
  const std::string& fleet_name)
{
  const auto fleet_state_it = _ingestor_common->fleet_states.find(fleet_name);
  if (fleet_state_it == _ingestor_common->fleet_states.end())
  {
    RCLCPP_WARN(_node->get_logger(),
      "No such fleet: [%s]", fleet_name.c_str());
    return false;
  }

  std::vector<gazebo::physics::ModelPtr> robot_model_list;
  for (const auto& rs : fleet_state_it->second->robots)
  {
    const auto r_model = _world->ModelByName(rs.name);
    if (r_model && !r_model->IsStatic())
      robot_model_list.push_back(r_model);
  }

  gazebo::physics::ModelPtr robot_model;
  if (!find_nearest_model(robot_model_list, robot_model) ||
    !robot_model)
  {
    RCLCPP_WARN(_node->get_logger(),
      "No nearby robots of fleet [%s] found.", fleet_name.c_str());
    return false;
  }

  if (!get_payload_model(robot_model, _ingested_model))
  {
    RCLCPP_WARN(_node->get_logger(),
      "No delivery item found on the robot: [%s]",
      robot_model->GetName());
    _ingested_model = nullptr;
    return false;
  }
  _ingested_model->SetWorldPose(_model->WorldPose());
  _ingestor_common->ingestor_filled = true;
  return true;
}

void TeleportIngestorPlugin::send_ingested_item_home()
{
  if (_ingestor_common->ingestor_filled)
  {
    const auto it = _ingestor_common->non_static_models_init_poses.find(
      _ingested_model->GetName());
    if (it == _ingestor_common->non_static_models_init_poses.end())
      _world->RemoveModel(_ingested_model);
    else
      _ingested_model->SetWorldPose(it->second);

    _ingestor_common->ingestor_filled = false; // Assumes ingestor can only hold 1 object at a time
  }
}

void TeleportIngestorPlugin::on_update()
{
  _ingestor_common->sim_time = _world->SimTime().Double();

  std::function<bool(const std::string&)> ingest_fn_cb =
    std::bind(&TeleportIngestorPlugin::ingest_from_nearest_robot,
      this, std::placeholders::_1);

  std::function<void(void)> send_ingested_item_home_cb =
    std::bind(&TeleportIngestorPlugin::send_ingested_item_home,
      this);

  _ingestor_common->on_update(ingest_fn_cb, send_ingested_item_home_cb);
}

void TeleportIngestorPlugin::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  _model = _parent;
  _world = _model->GetWorld();
  _ingested_model = nullptr;

  _node = gazebo_ros::Node::Get(_sdf);
  RCLCPP_INFO(_node->get_logger(), "Started TeleportIngestorPlugin node...");

  _ingestor_common->_guid = _model->GetName();
  _ingestor_common->init_ros_node(_node);

  // Keep track of all the non-static models
  auto model_list = _world->Models();
  for (const auto& m : model_list)
  {
    std::string m_name = m->GetName();
    if (m && !(m->IsStatic()) && m_name != _model->GetName())
      _ingestor_common->non_static_models_init_poses[m_name] = m->WorldPose();
  }

  _ingestor_common->current_state.guid = _ingestor_common->_guid;
  _ingestor_common->current_state.mode = IngestorState::IDLE;

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&TeleportIngestorPlugin::on_update, this));
}

TeleportIngestorPlugin::TeleportIngestorPlugin()
: _ingestor_common(std::make_unique<TeleportIngestorCommon>())
{
}

TeleportIngestorPlugin::~TeleportIngestorPlugin()
{
  rclcpp::shutdown();
}

} // namespace rmf_gazebo_plugins

GZ_REGISTER_MODEL_PLUGIN(rmf_gazebo_plugins::TeleportIngestorPlugin)
