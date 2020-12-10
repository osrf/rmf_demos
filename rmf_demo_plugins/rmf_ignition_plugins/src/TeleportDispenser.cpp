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
#include <unordered_map>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>

#include <ignition/math/AxisAlignedBox.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>

#include <rmf_plugins_common/dispenser_common.hpp>
#include <rmf_plugins_common/utils.hpp>

using namespace ignition::gazebo;
using namespace rmf_dispenser_common;
using namespace rmf_plugins_utils;

namespace rmf_ignition_plugins {

class IGNITION_GAZEBO_VISIBLE TeleportDispenserPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStateIt =
    std::unordered_map<std::string, FleetState::UniquePtr>::iterator;

  TeleportDispenserPlugin();
  ~TeleportDispenserPlugin();
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  // Stores params representing state of Dispenser, and handles the main dispenser logic
  std::unique_ptr<TeleportDispenserCommon> _dispenser_common;

  ignition::transport::Node _ign_node;
  ignition::transport::Node::Publisher _item_dispensed_pub;

  Entity _dispenser;
  Entity _item_en; // Item that dispenser may contain
  ignition::math::AxisAlignedBox _dispenser_vicinity_box;

  bool tried_fill_dispenser = false; // Set to true if fill_dispenser() has been called at least once

  rclcpp::Node::SharedPtr _ros_node;

  SimEntity find_nearest_model(
    EntityComponentManager& ecm,
    const std::vector<SimEntity>& entities, bool& found) const;
  void place_on_entity(EntityComponentManager& ecm,
    const SimEntity& obj, const Entity& to_move);
  void fill_robot_list(EntityComponentManager& ecm,
    FleetStateIt fleet_state_it, std::vector<SimEntity>& robot_list);
  bool dispense_on_nearest_robot(EntityComponentManager& ecm,
    const std::string& fleet_name);
  void fill_dispenser(EntityComponentManager& ecm);
  void create_dispenser_bounding_box(EntityComponentManager& ecm);
};

TeleportDispenserPlugin::TeleportDispenserPlugin()
: _dispenser_common(std::make_unique<TeleportDispenserCommon>())
{
}

TeleportDispenserPlugin::~TeleportDispenserPlugin()
{
  rclcpp::shutdown();
}

SimEntity TeleportDispenserPlugin::find_nearest_model(
  EntityComponentManager& ecm,
  const std::vector<SimEntity>& entities,
  bool& found) const
{
  double nearest_dist = 1e6;
  SimEntity robot_entity(0); // Placeholder value
  const auto dispenser_pos =
    ecm.Component<components::Pose>(_dispenser)->Data().Pos();

  for (const auto& sim_obj : entities)
  {
    Entity en = sim_obj.get_entity();
    std::string name = ecm.Component<components::Name>(en)->Data();
    if (name == _dispenser_common->guid)
      continue;

    const auto en_pos = ecm.Component<components::Pose>(en)->Data().Pos();
    const double dist = en_pos.Distance(dispenser_pos);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      robot_entity = sim_obj;
      found = true;
    }
  }
  return robot_entity;
}

// Move entity `to_move` onto `base_obj`
void TeleportDispenserPlugin::place_on_entity(EntityComponentManager& ecm,
  const SimEntity& base_obj, const Entity& to_move)
{
  const Entity base = base_obj.get_entity();
  auto new_pose = ecm.Component<components::Pose>(base)->Data();

  // Make service request to Slotcar to get its height instead of accessing
  // it's AABB component directly
  ignition::msgs::Entity req;
  req.set_id(base);

  const unsigned int timeout = 5000;
  bool result = false;
  ignition::msgs::Double rep;
  const std::string height_srv_name = "/slotcar_height_" + std::to_string(base);

  bool executed = _ign_node.Request(height_srv_name, req, timeout, rep, result);
  if (executed && result)
  {
    // Assumes that the base pose's Z value refers to bottom of object
    new_pose.SetZ(ecm.Component<components::Pose>(base)->Data().Z()
      + rep.data());
  }
  else
  {
    RCLCPP_WARN(
      _dispenser_common->ros_node->get_logger(),
      "Either base entity or item to be dispensed does not have an AxisAlignedBox component. \
      Attempting to dispense item to approximate location.");
    new_pose += ignition::math::Pose3<double>(0, 0, 0.5, 0, 0, 0);
  }

  auto cmd = ecm.Component<components::WorldPoseCmd>(to_move);
  if (!cmd)
  {
    ecm.CreateComponent(to_move,
      components::WorldPoseCmd(ignition::math::Pose3<double>()));
  }
  ecm.Component<components::WorldPoseCmd>(to_move)->Data() = new_pose;

  // For Ignition slotcar plugin to know when an item has been dispensed to it
  // Necessary for TPE Plugin
  ignition::msgs::UInt64_V dispense_msg;
  dispense_msg.add_data(google::protobuf::uint64(base)); //Conversion from Entity -> uint64
  dispense_msg.add_data(google::protobuf::uint64(to_move));
  _item_dispensed_pub.Publish(dispense_msg);
}

void TeleportDispenserPlugin::fill_robot_list(EntityComponentManager& ecm,
  FleetStateIt fleet_state_it, std::vector<SimEntity>& robot_list)
{
  for (const auto& rs : fleet_state_it->second->robots)
  {
    std::vector<Entity> entities =
      ecm.EntitiesByComponents(components::Name(rs.name),
        components::Model(), components::Static(false));
    for (Entity& en : entities)
    {
      robot_list.push_back(SimEntity(en));
    }
  }
}

// Searches vicinity of Dispenser for closest valid item. If found, the item is assigned to `_item_en`
void TeleportDispenserPlugin::fill_dispenser(EntityComponentManager& ecm)
{
  const auto dispenser_pos =
    ecm.Component<components::Pose>(_dispenser)->Data().Pos();

  double nearest_dist = 1.0;
  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity& en,
    const components::Model*,
    const components::Name* name,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      if (!is_static->Data() && name->Data() != _dispenser_common->guid)
      {
        const auto dist = pose->Data().Pos().Distance(dispenser_pos);

        if (dist < nearest_dist
        && _dispenser_vicinity_box.Contains(pose->Data().Pos()))
        {
          _item_en = en;
          nearest_dist = dist;
          _dispenser_common->dispenser_filled = true;
          _dispenser_common->item_en_found = true;
        }
      }
      return true;
    });

  if (!_dispenser_common->dispenser_filled)
  {
    RCLCPP_WARN(_dispenser_common->ros_node->get_logger(),
      "Could not find dispenser item model within 1 meter, "
      "this dispenser will not be operational");
  }
  else
  {
    RCLCPP_INFO(_dispenser_common->ros_node->get_logger(),
      "Found dispenser item: [%s]",
      ecm.Component<components::Name>(_item_en)->Data().c_str());
  }
}

void TeleportDispenserPlugin::create_dispenser_bounding_box(
  EntityComponentManager& ecm)
{
  const auto dispenser_pos =
    ecm.Component<components::Pose>(_dispenser)->Data().Pos();
  ignition::math::Vector3d corner_1(dispenser_pos.X() - 0.05,
    dispenser_pos.Y() - 0.05, dispenser_pos.Z() - 0.05);
  ignition::math::Vector3d corner_2(dispenser_pos.X() + 0.05,
    dispenser_pos.Y() + 0.05, dispenser_pos.Z() + 0.05);
  _dispenser_vicinity_box = ignition::math::AxisAlignedBox(corner_1, corner_2);
}

void TeleportDispenserPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>&,
  EntityComponentManager& ecm, EventManager&)
{
  char const** argv = NULL;
  if (!rclcpp::ok())
    rclcpp::init(0, argv);

  _dispenser = entity;
  _dispenser_common->guid =
    ecm.Component<components::Name>(_dispenser)->Data();
  ignwarn << "Initializing plugin with name " << _dispenser_common->guid <<
    std::endl;

  _ros_node = std::make_shared<rclcpp::Node>(_dispenser_common->guid);
  _dispenser_common->init_ros_node(_ros_node);
  RCLCPP_INFO(_dispenser_common->ros_node->get_logger(),
    "Started TeleportIngestorPlugin node...");

  // Needed for TPE plugin, so that subscriber knows when to move a payload that
  // has been dispensed to it
  _item_dispensed_pub = _ign_node.Advertise<ignition::msgs::UInt64_V>(
    "/item_dispensed");
  if (!_item_dispensed_pub)
  {
    ignwarn << "Error advertising topic [/item_dispensed]" << std::endl;
  }

  create_dispenser_bounding_box(ecm);
}

void TeleportDispenserPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  _dispenser_common->sim_time =
    std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count();
  // TODO parallel thread executor?
  rclcpp::spin_some(_dispenser_common->ros_node);

  // Set item that the Dispenser will be configured to dispense. Do this only on first PreUpdate() call.
  // Happens here and not in Configure() to allow for all models to load
  if (!tried_fill_dispenser)
  {
    fill_dispenser(ecm);
    tried_fill_dispenser = true;
  }

  std::function<void(FleetStateIt,
    std::vector<SimEntity>&)> fill_robot_list_cb =
    std::bind(&TeleportDispenserPlugin::fill_robot_list, this,
      std::ref(ecm), std::placeholders::_1, std::placeholders::_2);

  std::function<SimEntity(const std::vector<SimEntity>&,
    bool&)> find_nearest_model_cb =
    std::bind(&TeleportDispenserPlugin::find_nearest_model, this,
      std::ref(ecm), std::placeholders::_1, std::placeholders::_2);

  std::function<void(const SimEntity&)> place_on_entity_cb =
    std::bind(&TeleportDispenserPlugin::place_on_entity, this,
      std::ref(ecm), std::placeholders::_1, _item_en);

  std::function<bool(void)> check_filled_cb = [&]()
    {
      return _dispenser_vicinity_box.Contains(
        ecm.Component<components::Pose>(_item_en)->Data().Pos());
    };

  _dispenser_common->on_update(fill_robot_list_cb, find_nearest_model_cb,
    place_on_entity_cb, check_filled_cb);
}

IGNITION_ADD_PLUGIN(
  TeleportDispenserPlugin,
  System,
  TeleportDispenserPlugin::ISystemConfigure,
  TeleportDispenserPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(TeleportDispenserPlugin, "teleport_dispenser")

} // namespace rmf_ignition_plugins