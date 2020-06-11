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

class TeleportDispenserPlugin : public gazebo::ModelPlugin
{

public:

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

private:

  std::string _guid;
  double _last_pub_time = 0.0;
  bool _item_dispensed = false;
  bool _load_complete = false;

  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  gazebo::physics::ModelPtr _item_model;
  gazebo::physics::WorldPtr _world;

  ignition::math::Box _dispenser_vicinity_box;

  gazebo_ros::Node::SharedPtr _node;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;

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

  bool find_nearest_non_static_model_name(
    const std::vector<gazebo::physics::ModelPtr>& models,
    std::string& nearest_model_name) const
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
        nearest_model_name = m->GetName();
        found = true;
      }
    }
    return found;
  }

  void dispense_on_nearest_robot(const std::string& fleet_name) const
  {
    if (!_item_model)
      return;

    const auto fleet_state_it = _fleet_states.find(fleet_name);
    if (fleet_state_it == _fleet_states.end())
    {
      RCLCPP_WARN(_node->get_logger(),
        "No such fleet: [%s]", fleet_name.c_str());
      return;
    }

    std::vector<gazebo::physics::ModelPtr> robot_models;
    for (const auto& rs : fleet_state_it->second->robots)
    {
      const auto r_model = _world->ModelByName(rs.name);
      if (r_model)
        robot_models.push_back(r_model);
    }

    std::string nearest_robot_model_name;
    if (!find_nearest_non_static_model_name(
        robot_models, nearest_robot_model_name))
    {
      RCLCPP_WARN(_node->get_logger(),
        "No near robots of fleet [%s] found.", fleet_name.c_str());
      return;
    }
    _item_model->PlaceOnEntity(nearest_robot_model_name);
  }

  void fleet_state_cb(FleetState::UniquePtr msg)
  {
    _fleet_states[msg->name] = std::move(msg);
  }

  void send_dispenser_response(
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

    if (msg->target_guid == _guid && !_item_dispensed)
    {
      const auto it = _past_request_guids.find(request_guid);
      if (it != _past_request_guids.end())
      {
        if (it->second)
        {
          RCLCPP_WARN(_node->get_logger(),
            "Request already succeeded: [%s]", request_guid);
          send_dispenser_response(request_guid, DispenserResult::SUCCESS);
        }
        else
        {
          RCLCPP_WARN(_node->get_logger(),
            "Request already failed: [%s]", request_guid);
          send_dispenser_response(request_guid, DispenserResult::FAILED);
        }
        return;
      }

      send_dispenser_response(request_guid, DispenserResult::ACKNOWLEDGED);

      RCLCPP_INFO(_node->get_logger(), "Dispensing item");
      dispense_on_nearest_robot(transporter_type);
      rclcpp::sleep_for(std::chrono::seconds(5));
      _item_dispensed = true;

      send_dispenser_response(request_guid, DispenserResult::SUCCESS);

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

      if (_item_dispensed &&
        _item_model &&
        _model->BoundingBox().Intersects(
          _item_model->BoundingBox()))
        _item_dispensed = false;
    }
  }

public:

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
  {
    _model = _parent;
    _world = _model->GetWorld();

    _node = gazebo_ros::Node::Get(_sdf);
    RCLCPP_INFO(_node->get_logger(), "Started TeleportDispenserPlugin node...");

    // find the dispenser item model, maximum distance 1 meter
    const auto dispenser_pos = _model->WorldPose().Pos();
    ignition::math::Vector3d corner_1 = dispenser_pos;
    corner_1.X(dispenser_pos.X() - 0.05);
    corner_1.Y(dispenser_pos.Y() - 0.05);
    corner_1.Z(dispenser_pos.Z() - 0.05);
    ignition::math::Vector3d corner_2 = dispenser_pos;
    corner_2.X(dispenser_pos.X() + 0.05);
    corner_2.Y(dispenser_pos.Y() + 0.05);
    corner_2.Z(dispenser_pos.Z() + 0.05);
    _dispenser_vicinity_box = ignition::math::Box(corner_1, corner_2);

    auto model_list = _world->Models();
    double nearest_dist = 1.0;
    for (const auto& m : model_list)
    {
      if (!m || m->IsStatic() || m->GetName() == _model->GetName())
        continue;

      const double dist = m->WorldPose().Pos().Distance(dispenser_pos);
      if (dist < nearest_dist &&
        _dispenser_vicinity_box.Intersects(m->BoundingBox()))
      {
        _item_model = m;
        nearest_dist = dist;
      }
    }

    if (!_item_model)
    {
      RCLCPP_WARN(_node->get_logger(),
        "Could not find dispenser item model within 1 meter, "
        "this dispenser will not be operational");
      return;
    }

    RCLCPP_INFO(_node->get_logger(),
      "Found dispenser item: [%s]", _item_model->GetName().c_str());

    _guid = _model->GetName();

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
      std::bind(&TeleportDispenserPlugin::on_update, this));
    _load_complete = true;
  }

  ~TeleportDispenserPlugin()
  {
    if (_load_complete)
      rclcpp::shutdown();
  }

};

} // namespace rmf_gazebo_plugins

GZ_REGISTER_MODEL_PLUGIN(rmf_gazebo_plugins::TeleportDispenserPlugin)
