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
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <ignition/math/Vector3.hh>

#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>

namespace rmf_gazebo_plugins {

class TeleportIngestorPlugin : public gazebo::ModelPlugin
{

public:

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  using Pose3d = ignition::math::Pose3d;

private:

  std::string _guid;
  double _last_pub_time = 0.0;
  bool _load_complete = false;

  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  gazebo::physics::ModelPtr _ingested_model;
  gazebo::physics::WorldPtr _world;

  gazebo_ros::Node::SharedPtr _node;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;

  std::unordered_map<std::string, ignition::math::Pose3d>
  _non_static_models_init_poses;

  std::unordered_map<std::string, FleetState::UniquePtr> _fleet_states;

  std::unordered_map<std::string, bool> _past_request_guids;

  DispenserState _current_state;

  rclcpp::Time simulation_now() const
  {
    const double t = _model->GetWorld()->SimTime().Double();
    const int32_t t_sec = static_cast<int32_t>(t);
    const uint32_t t_nsec =
      static_cast<uint32_t>((t-static_cast<double>(t_sec)) * 1e9);
    return rclcpp::Time{t_sec, t_nsec, RCL_ROS_TIME};
  }

  bool find_nearest_non_static_model(
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

  bool get_payload_model(
    const gazebo::physics::ModelPtr& robot_model,
    gazebo::physics::ModelPtr& payload_model) const
  {
    if (!robot_model)
      return false;

    const ignition::math::Box robot_collision_bb = robot_model->BoundingBox();
    ignition::math::Vector3d max_corner = robot_collision_bb.Max();

    // create a new bounding box extended slightly in the Z direction
    max_corner.Z(max_corner.Z() + 0.1);
    const ignition::math::Box vicinity_box(
      robot_collision_bb.Min(), max_corner);

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

  void ingest_from_nearest_robot(const std::string& fleet_name)
  {
    const auto fleet_state_it = _fleet_states.find(fleet_name);
    if (fleet_state_it == _fleet_states.end())
    {
      RCLCPP_WARN(_node->get_logger(),
        "No such fleet: [%s]", fleet_name.c_str());
      return;
    }

    std::vector<gazebo::physics::ModelPtr> robot_model_list;
    for (const auto& rs : fleet_state_it->second->robots)
    {
      const auto r_model = _world->ModelByName(rs.name);
      if (r_model)
        robot_model_list.push_back(r_model);
    }

    gazebo::physics::ModelPtr robot_model;
    if (!find_nearest_non_static_model(robot_model_list, robot_model) ||
      !robot_model)
    {
      RCLCPP_WARN(_node->get_logger(),
        "No nearby robots of fleet [%s] found.", fleet_name.c_str());
      return;
    }

    if (!get_payload_model(robot_model, _ingested_model))
    {
      RCLCPP_WARN(_node->get_logger(),
        "No delivery item found on the robot: [%s]",
        robot_model->GetName());
      _ingested_model = nullptr;
      return;
    }
    _ingested_model->SetWorldPose(_model->WorldPose());
  }

  void send_ingested_item_home()
  {
    if (_ingested_model)
    {
      const auto it =
        _non_static_models_init_poses.find(_ingested_model->GetName());
      if (it == _non_static_models_init_poses.end())
        _world->RemoveModel(_ingested_model);
      else
        _ingested_model->SetWorldPose(it->second);

      _ingested_model = nullptr;
    }
  }

  void fleet_state_cb(FleetState::UniquePtr msg)
  {
    _fleet_states[msg->name] = std::move(msg);
  }

  void send_ingestor_response(
    const std::string& request_guid, uint8_t status) const
  {
    DispenserResult response;
    response.time = simulation_now();
    response.request_guid = request_guid;
    response.source_guid = _guid;
    response.status = status;
    _result_pub->publish(response);
  }

  void dispenser_request_cb(DispenserRequest::UniquePtr msg)
  {
    // TODO: the message field should use fleet name instead
    const auto transporter_type = msg->transporter_type;
    const auto request_guid = msg->request_guid;

    if (_guid == msg->target_guid && !_ingested_model)
    {
      const auto it = _past_request_guids.find(request_guid);
      if (it != _past_request_guids.end())
      {
        if (it->second)
        {
          RCLCPP_WARN(_node->get_logger(),
            "Request already succeeded: [%s]", request_guid);
          send_ingestor_response(request_guid, DispenserResult::SUCCESS);
        }
        else
        {
          RCLCPP_WARN(_node->get_logger(),
            "Request already failed: [%s]", request_guid);
          send_ingestor_response(request_guid, DispenserResult::FAILED);
        }
        return;
      }

      send_ingestor_response(request_guid, DispenserResult::ACKNOWLEDGED);

      RCLCPP_INFO(_node->get_logger(), "Ingesting item");
      ingest_from_nearest_robot(transporter_type);

      send_ingestor_response(request_guid, DispenserResult::SUCCESS);

      rclcpp::sleep_for(std::chrono::seconds(10));
      send_ingested_item_home();

      // There are currently no cases to publish a FAILED result yet
    }
  }

  void on_update()
  {
    if (!_load_complete)
      return;

    const double t = _world->SimTime().Double();
    if (t - _last_pub_time >= 2.0)
    {
      _last_pub_time = t;
      const auto now = simulation_now();

      _current_state.time = now;
      _current_state.mode = DispenserState::IDLE;
      _state_pub->publish(_current_state);
    }
  }

public:

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
  {
    _model = _parent;
    _world = _model->GetWorld();
    _ingested_model = nullptr;

    _node = gazebo_ros::Node::Get(_sdf);
    RCLCPP_INFO(_node->get_logger(), "Started TeleportIngestorPlugin node...");

    _guid = _model->GetName();

    // Keep track of all the non-static models
    auto model_list = _world->Models();
    for (const auto& m : model_list)
    {
      std::string m_name = m->GetName();
      if (m && !(m->IsStatic()) && m_name != _model->GetName())
        _non_static_models_init_poses[m_name] = m->WorldPose();
    }

    _fleet_state_sub = _node->create_subscription<FleetState>(
      "/fleet_states",
      rclcpp::SystemDefaultsQoS(),
      [&](FleetState::UniquePtr msg)
      {
        fleet_state_cb(std::move(msg));
      });

    _state_pub = _node->create_publisher<DispenserState>(
      "/dispenser_states", 10);

    _request_sub = _node->create_subscription<DispenserRequest>(
      "/dispenser_requests",
      rclcpp::SystemDefaultsQoS(),
      [&](DispenserRequest::UniquePtr msg)
      {
        dispenser_request_cb(std::move(msg));
      });

    _result_pub = _node->create_publisher<DispenserResult>(
      "/dispenser_results", 10);

    _current_state.guid = _guid;
    _current_state.mode = DispenserState::IDLE;

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&TeleportIngestorPlugin::on_update, this));
    _load_complete = true;
  }

  ~TeleportIngestorPlugin()
  {
    if (_load_complete)
      rclcpp::shutdown();
  }

};

} // namespace rmf_gazebo_plugins

GZ_REGISTER_MODEL_PLUGIN(rmf_gazebo_plugins::TeleportIngestorPlugin)
