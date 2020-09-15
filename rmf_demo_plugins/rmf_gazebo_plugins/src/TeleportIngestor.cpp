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

#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <rmf_plugins_common/ingestor_common.hpp>
#include <rmf_plugins_common/utils.hpp>

using namespace rmf_plugins_utils;
using namespace rmf_ingestor_common;

namespace rmf_gazebo_plugins {

class TeleportIngestorPlugin : public gazebo::ModelPlugin
{

public:
  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStateIt =
    std::unordered_map<std::string, FleetState::UniquePtr>::iterator;

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

  SimEntity find_nearest_model(const std::vector<SimEntity>& models,
    bool& found) const;
  bool get_payload_model(const SimEntity& robot_model,
    gazebo::physics::ModelPtr& payload_model) const;
  void fill_robot_list(FleetStateIt fleet_state_it,
    std::vector<SimEntity>& robot_model_list);
  void transport_model();
  void send_ingested_item_home();
  void on_update();
};

SimEntity TeleportIngestorPlugin::find_nearest_model(
  const std::vector<SimEntity>& models,
  bool& found) const
{
  double nearest_dist = 1e6;
  SimEntity nearest_model("");

  for (const SimEntity& sim_obj : models)
  {
    if (sim_obj.get_name() == _model->GetName())
      continue;

    // If `models` has been generated with `fill_robot_list`, it is
    // guaranteed to have a valid name field
    gazebo::physics::EntityPtr m = _world->EntityByName(sim_obj.get_name());
    const double dist =
      m->WorldPose().Pos().Distance(_model->WorldPose().Pos());
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      nearest_model = sim_obj;
      found = true;
    }
  }
  return nearest_model;
}

// Identifies item to ingest and assigns it to `payload_model`
bool TeleportIngestorPlugin::get_payload_model(
  const SimEntity& robot_sim_entity,
  gazebo::physics::ModelPtr& payload_model) const
{
  const auto robot_model = _world->EntityByName(robot_sim_entity.get_name());
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
    if (!m || m->IsStatic() ||
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

void TeleportIngestorPlugin::fill_robot_list(
  FleetStateIt fleet_state_it, std::vector<SimEntity>& robot_model_list)
{
  for (const auto& rs : fleet_state_it->second->robots)
  {
    const auto r_model = _world->ModelByName(rs.name);
    if (r_model && !r_model->IsStatic())
      robot_model_list.push_back(SimEntity(r_model->GetName()));
  }
}

// Moves the identified item to ingest from its current position to the ingestor
void TeleportIngestorPlugin::transport_model()
{
  _ingested_model->SetWorldPose(_model->WorldPose());
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
      _ingested_model->SetWorldPose(convert_to_pose<ignition::math::Pose3d>(
          it->second));

    _ingestor_common->ingestor_filled = false; // Assumes ingestor can only hold 1 object at a time
  }
}

void TeleportIngestorPlugin::on_update()
{
  _ingestor_common->sim_time = _world->SimTime().Double();

  std::function<void(FleetStateIt,
    std::vector<rmf_plugins_utils::SimEntity>&)> fill_robot_list_cb =
    std::bind(&TeleportIngestorPlugin::fill_robot_list, this,
      std::placeholders::_1, std::placeholders::_2);

  std::function<SimEntity(const std::vector<rmf_plugins_utils::SimEntity>&,
    bool&)> find_nearest_model_cb =
    std::bind(&TeleportIngestorPlugin::find_nearest_model, this,
      std::placeholders::_1, std::placeholders::_2);

  std::function<bool(const SimEntity&)> get_payload_model_cb =
    std::bind(&TeleportIngestorPlugin::get_payload_model, this,
      std::placeholders::_1, std::ref(_ingested_model));

  std::function<void()> transport_model_cb =
    std::bind(&TeleportIngestorPlugin::transport_model, this);

  std::function<void(void)> send_ingested_item_home_cb =
    std::bind(&TeleportIngestorPlugin::send_ingested_item_home, this);

  _ingestor_common->on_update(fill_robot_list_cb, find_nearest_model_cb,
    get_payload_model_cb, transport_model_cb, send_ingested_item_home_cb);
}

void TeleportIngestorPlugin::Load(gazebo::physics::ModelPtr _parent,
  sdf::ElementPtr _sdf)
{
  _model = _parent;
  _world = _model->GetWorld();
  _ingested_model = nullptr;

  _ingestor_common->_guid = _model->GetName();
  _ingestor_common->init_ros_node(gazebo_ros::Node::Get(_sdf));
  RCLCPP_INFO(_ingestor_common->ros_node->get_logger(),
    "Started TeleportIngestorPlugin node...");

  // Keep track of all the non-static models
  auto model_list = _world->Models();
  for (const auto& m : model_list)
  {
    std::string m_name = m->GetName();
    if (m && !(m->IsStatic()) && m_name != _model->GetName())
    {
      _ingestor_common->non_static_models_init_poses[m_name] = convert_pose(
        m->WorldPose());
    }
  }

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
