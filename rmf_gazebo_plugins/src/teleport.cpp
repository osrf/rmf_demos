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

#include <mutex>
#include <unordered_map>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <ignition/math/Vector3.hh>

#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>

namespace rmf_gazebo_plugins {

class TeleportPlugin : public gazebo::ModelPlugin
{
public: 

  gazebo::event::ConnectionPtr _update_connection;

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  using Pose3d = ignition::math::Pose3d;

  // Pointer to the model
  gazebo::physics::ModelPtr _model;
  gazebo::physics::WorldPtr _world;
  gazebo_ros::Node::SharedPtr _node;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;

  std::unordered_map<std::string, FleetState::UniquePtr> _fleet_states;

  DispenserState _load_dispenser_state;
  DispenserState _unload_dispenser_state;

  double _last_pub_time = 0.0;

  bool _load_complete = false;

  // The guid of the loading and unloading dispensers
  std::string _load_guid;
  std::string _unload_guid;
  std::vector<std::string> _unload_model_names;

  Pose3d _initial_pose;

  bool _object_loaded = false;

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
  {
    // Store the pointer to the model
    _model = _parent;
    _world = _model->GetWorld();

    // Get the required sdf parameters
    get_sdf_param_required<std::string>(_sdf, "load_guid", _load_guid);
    get_sdf_param_required<std::string>(_sdf, "unload_guid", _unload_guid);
    _unload_model_names = 
        get_sdf_params_if_available<std::string>(_sdf, "unload_model");

    _node = gazebo_ros::Node::Get(_sdf);
    std::cout << "Started teleport_plugin node..." <<std::endl;

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
    
    _load_dispenser_state.guid = _load_guid;
    _load_dispenser_state.mode = DispenserState::IDLE;
    _unload_dispenser_state.guid = _unload_guid;
    _unload_dispenser_state.mode = DispenserState::IDLE;

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&TeleportPlugin::on_update, this));

    // Fixed location for "reloading" payloads
    _initial_pose = _model->WorldPose();
    _load_complete = true;
  }

  void fleet_state_cb(FleetState::UniquePtr msg)
  {
    _fleet_states[msg->name] = std::move(msg);
  }

  void dispenser_request_cb(DispenserRequest::UniquePtr msg)
  {
    // TODO: the message field should use fleet name instead
    auto transporter_type = msg->transporter_type;
    auto dispenser_guid = msg->target_guid;
    
    DispenserResult response;
    response.request_guid = msg->request_guid;

    if (dispenser_guid == _load_guid && !_object_loaded)
    {
      response.time = _node->now();
      response.source_guid = _load_guid;
      response.status = DispenserResult::ACKNOWLEDGED;
      _result_pub->publish(response);

      RCLCPP_INFO(_node->get_logger(), "Loading object");
      load_on_nearest_robot(transporter_type);
      _object_loaded = true;

      response.time = _node->now();
      response.status = DispenserResult::SUCCESS;
      _result_pub->publish(response);
    }
    else if (dispenser_guid == _unload_guid && _object_loaded)
    {
      response.time = _node->now();
      response.source_guid = _unload_guid;
      response.status = DispenserResult::ACKNOWLEDGED;
      _result_pub->publish(response);

      RCLCPP_INFO(_node->get_logger(), "Unloading object");
      unload_on_nearest_target();
      _object_loaded = false;

      response.time = _node->now();
      response.status = DispenserResult::SUCCESS;
      _result_pub->publish(response);
      
      // TODO(Aaron): do this in a separate thread so state publishing continues
      // Hard coded: Leave object at goal location for 2.0 second, then
      // teleport it back to initial ( pre pickup  ) location
      rclcpp::sleep_for(std::chrono::seconds(2));
      _model->SetWorldPose(_initial_pose);
      _object_loaded = false;
    }
  }

  void load_on_nearest_robot(const std::string& fleet_name)
  {    
    auto fleet_state = _fleet_states.find(fleet_name);
    if (fleet_state == _fleet_states.end())
    {
      RCLCPP_WARN(_node->get_logger(), 
          "No such fleet: [%s]", fleet_name.c_str());
      return;
    }

    double nearest_robot_distance = 1e6;
    std::string nearest_robot_name;
    for (auto rs : fleet_state->second->robots)
    {
      auto rmodel = _world->ModelByName(rs.name);
      if (!rmodel)
        return;
      double dist = 
          rmodel->WorldPose().Pos().Distance(_model->WorldPose().Pos());
      if (dist < nearest_robot_distance)
      {
        nearest_robot_distance = dist;
        nearest_robot_name = rs.name;
      }
    }
    
    if (nearest_robot_name.empty())
    {
      RCLCPP_WARN(_node->get_logger(),
          "No near robots of fleet [%s] found.", fleet_name.c_str());
      return;
    }

    _model->PlaceOnEntity(nearest_robot_name);
  }

  void unload_on_nearest_target()
  {
    double nearest_unload_model_distance = 1e6;
    std::string nearest_unload_model_name;
    for (auto umodel_name : _unload_model_names)
    {
      auto umodel = _world->ModelByName(umodel_name);
      if (!umodel)
        return;
      double dist = 
          umodel->WorldPose().Pos().Distance(_model->WorldPose().Pos());
      if (dist < nearest_unload_model_distance)
      {
        nearest_unload_model_distance = dist;
        nearest_unload_model_name = umodel_name;
      }
    }

    if (nearest_unload_model_name.empty())
    {
      RCLCPP_WARN(_node->get_logger(), "No near unloading model found.");
      return;
    }

    _model->PlaceOnEntity(nearest_unload_model_name);
  }

  void on_update()
  {
    if (!_load_complete)
      return;

    const double t = _model->GetWorld()->SimTime().Double();
    if (t - _last_pub_time >= 2.0)
    {
      _last_pub_time = t;

      _load_dispenser_state.time = _node->now();
      _load_dispenser_state.mode = 
          _load_dispenser_state.request_guid_queue.empty() ?
          DispenserState::IDLE : DispenserState::BUSY;
      _state_pub->publish(_load_dispenser_state);

      _unload_dispenser_state.time = _node->now();
      _unload_dispenser_state.mode =
          _unload_dispenser_state.request_guid_queue.empty() ?
          DispenserState::IDLE : DispenserState::BUSY;
      _state_pub->publish(_unload_dispenser_state);
    }
  }

  ~TeleportPlugin()
  {
    if (_load_complete)
    {
      rclcpp::shutdown();
    }
  }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(TeleportPlugin)

} // namespace rmf_gazebo_plugins
