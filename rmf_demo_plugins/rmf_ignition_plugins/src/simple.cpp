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

#include <rmf_plugins_common/utils.hpp>

using namespace ignition::gazebo;

namespace rmf_ignition_plugins {

class IGNITION_GAZEBO_VISIBLE SimplePlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  SimplePlugin();
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  rclcpp::Node::SharedPtr _ros_node;
  Entity _en;
};

SimplePlugin::SimplePlugin()
{
  std::cout << "Creating simple plugin " << std::endl;
}

void SimplePlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  EntityComponentManager& ecm, EventManager&)
{
  _en = entity;
  std::cout << "FOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO " << std::endl;
}

void SimplePlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  std::cout << "Foo " << std::endl;
}

IGNITION_ADD_PLUGIN(
  SimplePlugin,
  System,
  SimplePlugin::ISystemConfigure,
  SimplePlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(SimplePlugin, "simple")

} // namespace rmf_ignition_plugins
