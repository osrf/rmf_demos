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
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <memory>

#include <Eigen/Geometry>

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <building_map_msgs/msg/building_map.hpp>
#include <building_map_msgs/msg/level.hpp>
#include <building_map_msgs/msg/graph.hpp>

#include <rmf_plugins_common/utils.hpp>
#include <rmf_plugins_common/readonly_common.hpp>

using namespace ignition::gazebo;

namespace rmf_ignition_plugins {

class IGNITION_GAZEBO_VISIBLE ReadonlyPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  ReadonlyPlugin();
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  std::unique_ptr<rmf_readonly_common::ReadonlyCommon> _readonly_common;
  rclcpp::Node::SharedPtr _ros_node;
  Entity _en;
};

ReadonlyPlugin::ReadonlyPlugin()
: _readonly_common(std::make_unique<rmf_readonly_common::ReadonlyCommon>())
{
}

void ReadonlyPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  EntityComponentManager& ecm, EventManager&)
{
  _en = entity;

  if (ecm.EntityHasComponentType(_en, components::Name().TypeId()))
  {
    _readonly_common->set_name(ecm.Component<components::Name>(_en)->Data());
  }

  _readonly_common->read_sdf(sdf);
  _ros_node = std::make_shared<rclcpp::Node>(_readonly_common->get_name());
  _readonly_common->init(_ros_node);
}

void ReadonlyPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  auto pose = rmf_plugins_utils::convert_pose(
    ecm.Component<components::Pose>(_en)->Data());
  auto sim_time =
    std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count();
  rclcpp::spin_some(_readonly_common->ros_node);
  _readonly_common->on_update(pose, sim_time);
}

IGNITION_ADD_PLUGIN(
  ReadonlyPlugin,
  System,
  ReadonlyPlugin::ISystemConfigure,
  ReadonlyPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(ReadonlyPlugin, "readonly")

} // namespace rmf_ignition_plugins
