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

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>

#include <ignition/math/Vector3.hh>

#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>

namespace rmf_gazebo_plugins
{

class TeleportPlugin : public gazebo::ModelPlugin
{
public:
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  using Pose3d          = ignition::math::Pose3d;

  // Pointer to the model
  gazebo::physics::ModelPtr _model;
  gazebo_ros::Node::SharedPtr _node;
  rclcpp::Subscription<DispenserResult>::SharedPtr _result_sub;
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
    std::cout << "Started teleport_plugin node..." << std::endl;

    _result_sub = _node->create_subscription<DispenserResult>(
        "/dispenser_results", rclcpp::SystemDefaultsQoS(),
        [&](DispenserResult::UniquePtr msg) { dispenser_result_cb(std::move(msg)); });

    _initial_pose = _model->WorldPose();

    _load_complete = true;
  }

  // Called by the world update start event
  void dispenser_result_cb(DispenserResult::UniquePtr msg)
  {
    auto status             = msg->status;
    std::string source_guid = msg->source_guid.c_str();

    if (source_guid == _load_guid && !_object_loaded)
    {
      RCLCPP_INFO(_node->get_logger(), "Loading object");
      _model->SetWorldPose(_load_pose);
      _object_loaded = true;
    }
    else if (source_guid == _unload_guid && _object_loaded)
    {
      RCLCPP_INFO(_node->get_logger(), "Unloading object");
      _model->SetWorldPose(_unload_pose);

      // BH: Hard coded: Leave object at goal location for 3.0 second, then
      // teleport
      // it back to initial ( pre pickup  ) location
      rclcpp::sleep_for(std::chrono::nanoseconds(3000000000));
      _model->SetWorldPose(_initial_pose);
      _object_loaded = false;
    }
    else
    {
      return;
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
