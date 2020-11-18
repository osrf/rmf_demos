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
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <memory>
#include <unordered_map>

#include <Eigen/Geometry>

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <building_map_msgs/msg/building_map.hpp>
#include <building_map_msgs/msg/level.hpp>
#include <building_map_msgs/msg/graph.hpp>

#include <rmf_plugins_common/utils.hpp>
#include <rmf_plugins_common/readonly_common.hpp>

class ReadonlyPlugin : public gazebo::ModelPlugin
{
public:
  ReadonlyPlugin();
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void OnUpdate();

private:
  std::unique_ptr<rmf_readonly_common::ReadonlyCommon> _readonly_common;
  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
};

ReadonlyPlugin::ReadonlyPlugin()
: _readonly_common(std::make_unique<rmf_readonly_common::ReadonlyCommon>())
{
}

void ReadonlyPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  _model = model;
  _readonly_common->set_name(_model->GetName());
  _readonly_common->read_sdf(sdf);
  _readonly_common->init(gazebo_ros::Node::Get(sdf));

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ReadonlyPlugin::OnUpdate, this));
}

void ReadonlyPlugin::OnUpdate()
{
  const auto& world = _model->GetWorld();
  auto pose = rmf_plugins_utils::convert_pose(_model->WorldPose());
  auto sim_time = world->SimTime().Double();
  _readonly_common->on_update(pose, sim_time);
}

GZ_REGISTER_MODEL_PLUGIN(ReadonlyPlugin)
