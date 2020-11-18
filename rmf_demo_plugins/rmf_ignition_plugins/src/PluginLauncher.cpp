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
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Events.hh>
#include <ignition/plugin/Register.hh>

#include <memory>

#include <sdf/sdf.hh>

using namespace ignition::gazebo;

namespace rmf_ignition_plugins {

// Plugin that launches its child ignition plugins. Useful when adding both
// Ignition and Gazebo plugins to the same sdf file. Serves as a workaround
// for including Ignition plugins that are part of the default installation,
// so that Gazebo classic does not attempt to load them and crash. With this
// plugin, Gazebo classic will simply issue an error when attempting to load
// the PluginLauncher and ignore any inner plugins instead of crashing.
class IGNITION_GAZEBO_VISIBLE PluginLauncher
  : public System,
  public ISystemConfigure
{
public:
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager&,
    EventManager& event_manager) override;
};

void PluginLauncher::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>& sdf,
  EntityComponentManager&, EventManager& event_manager)
{
  sdf::ElementPtr plugin_elem = sdf->GetElementImpl("plugin");
  while (plugin_elem)
  {
    if (plugin_elem->Get<std::string>("filename") != "__default__" &&
      plugin_elem->Get<std::string>("name") != "__default__")
    {
      // Assumes that parent is the entity to which plugin_elem must be attached
      sdf->GetParent()->InsertElement(plugin_elem);
      // Could still work without the following line
      event_manager.Emit<events::LoadPlugins>(entity, plugin_elem);
    }

    plugin_elem = plugin_elem->GetNextElement("plugin");
  }
}

IGNITION_ADD_PLUGIN(
  PluginLauncher,
  System,
  PluginLauncher::ISystemConfigure)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(PluginLauncher, "plugin_launcher")

} // namespace rmf_ignition_plugins
