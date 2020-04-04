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
#include <gazebo_ros/node.hpp>

#include <ignition/math/Vector3.hh>

#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>

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
  gazebo_ros::Node::SharedPtr _node;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;

  std::unordered_map<std::string, FleetState::UniquePtr> _fleet_states;

  double _last_pub_time = 0.0;

  bool _load_complete = false;

  // The guid of the loading and unloading dispensers
  std::string _load_guid;
  std::string _unload_guid;

  Pose3d _initial_pose;
  Pose3d _load_pose;
  Pose3d _unload_pose;

  bool _object_loaded = false;

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
  {
    // Store the pointer to the model
    _model = _parent;

    // Get the required sdf parameters
    get_sdf_param_required<std::string>(_sdf, "load_guid", _load_guid);
    get_sdf_param_required<std::string>(_sdf, "unload_guid", _unload_guid);
    get_sdf_param_required<Pose3d>(_sdf, "load_pose", _load_pose);
    get_sdf_param_required<Pose3d>(_sdf, "unload_pose", _unload_pose);

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
    auto dispenser_guid = msg->target_guid;
    if (dispenser_guid == _load_guid && !_object_loaded)
    {
      _object_loaded = true;
    }
    else if (dispenser_guid == _unload_guid && _object_loaded)
    {
      _object_loaded = false;
    }
    
    
    // TODO: the message field should use fleet name instead
    auto transporter_type = msg->transporter_type;


    auto status = msg->status;
    std::string source_guid = msg->source_guid.c_str();
  }

  // Called by the world update start event
  // void dispenser_result_cb(DispenserResult::UniquePtr msg)
  // {
  //   auto status = msg->status;
  //   std::string source_guid = msg->source_guid.c_str();

  //   if (source_guid == _load_guid && !_object_loaded)
  //   {
  //     RCLCPP_INFO(_node->get_logger(),  "Loading object");
  //     _model->SetWorldPose(_load_pose);
  //     _object_loaded = true;
  //   }
  //   else if (source_guid == _unload_guid && _object_loaded)
  //   {
  //     RCLCPP_INFO(_node->get_logger(),  "Unloading object");
  //     _model->SetWorldPose(_unload_pose);
  //     _object_loaded = false;

  //     // Hard coded: Leave object at goal location for 2.0 second, then
  //     // teleport it back to initial ( pre pickup  ) location
  //     rclcpp::sleep_for(std::chrono::seconds(2));
  //     _model->SetWorldPose(_initial_pose);
  //     _object_loaded = false;
  //   }
  //   else
  //   {
  //     return;
  //   } 
  // }

  void on_update()
  {
    if (!_load_complete)
      return;

    const double t = _model->GetWorld()->SimTime().Double();
    if (t - _last_pub_time >= 2.0)
    {
      _last_pub_time = t;

      // do stuff

      _state_pub->publish(_state);
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
