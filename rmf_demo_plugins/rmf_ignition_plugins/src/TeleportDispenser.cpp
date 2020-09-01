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

#include <sdf/Geometry.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/AxisAlignedBox.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Box.hh>
#include <ignition/math/AxisAlignedBox.hh>

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>

#include <rmf_plugins_common/dispenser_common.hpp>

// TODO remove this
using namespace ignition;
using namespace ignition::gazebo;
using namespace rmf_dispenser_common;

namespace rmf_ignition_plugins {

class IGNITION_GAZEBO_VISIBLE TeleportDispenserPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:

  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

  TeleportDispenserPlugin();
  ~TeleportDispenserPlugin();
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  std::unique_ptr<TeleportDispenserCommon> DispenserCommonPtr;

  Entity _dispenser;
  Entity _item_en; //item that dispenser may contain

  ignition::math::AxisAlignedBox _dispenser_vicinity_box;

  bool _load_complete = false;
  bool _item_en_found = false; // True if entity to be dispensed has been determined. Used to locate item in future
  bool tried_fill_dispenser = false; // Flag set to true if fill_dispenser() has been called

  rclcpp::Node::SharedPtr _ros_node;

  bool find_nearest_non_static_model(
    EntityComponentManager& ecm,
    const std::vector<Entity>& robot_model_entities,
    Entity& robot_entity) const;
  void dispense_on_nearest_robot(EntityComponentManager& ecm, const std::string& fleet_name);
  void fill_dispenser(EntityComponentManager& ecm);
  void create_dispenser_bounding_box(EntityComponentManager& ecm);
};

TeleportDispenserPlugin::TeleportDispenserPlugin()
: DispenserCommonPtr(std::make_unique<TeleportDispenserCommon>())
{
  // We do initialization only during ::Configure
}

TeleportDispenserPlugin::~TeleportDispenserPlugin()
{
}

bool TeleportDispenserPlugin::find_nearest_non_static_model(
  EntityComponentManager& ecm,
  const std::vector<Entity>& robot_model_entities,
  Entity& robot_entity) const
{
  double nearest_dist = 1e6;
  bool found = false;
  const auto dispenser_pos = ecm.Component<components::Pose>(_dispenser)->Data().Pos();

  for (const auto& en : robot_model_entities)
  {
    bool is_static = ecm.Component<components::Static>(en)->Data();
    std::string name = ecm.Component<components::Name>(en)->Data();

    if (!en || is_static || name == DispenserCommonPtr->_guid)
      continue;

    const auto en_pos = ecm.Component<components::Pose>(en)->Data().Pos();
    const double dist = en_pos.Distance(dispenser_pos);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      robot_entity = en;
      found = true;
    }
  }
  return found;
}

void TeleportDispenserPlugin::dispense_on_nearest_robot(EntityComponentManager& ecm, const std::string& fleet_name)
{
  if (!DispenserCommonPtr->_dispenser_filled)
    return;

  const auto fleet_state_it = DispenserCommonPtr->_fleet_states.find(fleet_name);
  if (fleet_state_it == DispenserCommonPtr->_fleet_states.end())
  {
    RCLCPP_WARN(DispenserCommonPtr->logger(),
      "No such fleet: [%s]", fleet_name.c_str());
    return;
  }
  std::vector<Entity> robot_model_list;
  for (const auto& rs : fleet_state_it->second->robots)
  {
    std::vector<Entity> entities = ecm.EntitiesByComponents(components::Name(rs.name), components::Model());
    robot_model_list.insert(robot_model_list.end(), entities.begin(), entities.end());
  }

  Entity robot_model;
  if (!find_nearest_non_static_model(ecm, robot_model_list, robot_model))
  {
    RCLCPP_WARN(DispenserCommonPtr->logger(),
      "No nearby robots of fleet [%s] found.", fleet_name.c_str());
    return;
  }

  // Move item from dispenser onto robot
  auto cmd = ecm.Component<components::WorldPoseCmd>(_item_en);
  if (!cmd) {
    ecm.CreateComponent(_item_en, components::WorldPoseCmd(ignition::math::Pose3<double>()));
  }
  auto new_pose = ecm.Component<components::Pose>(robot_model)->Data() + ignition::math::Pose3<double>(0,0,0.5,0,0,0);
  ecm.Component<components::WorldPoseCmd>(_item_en)->Data() = new_pose;
  DispenserCommonPtr->_dispenser_filled = false;
}

void TeleportDispenserPlugin::fill_dispenser(EntityComponentManager& ecm){
    const auto dispenser_pos = ecm.Component<components::Pose>(_dispenser)->Data().Pos();

    double nearest_dist = 1.0;
    ecm.Each<components::Model, components::Name, components::Pose, components::Static>(
      [&](const Entity& en,
      const components::Model*,
      const components::Name* name,
      const components::Pose* pose,
      const components::Static* is_static
      ) -> bool
      {
        if (!is_static->Data() && name->Data() != DispenserCommonPtr->_guid)
        {
          const auto dist = pose->Data().Pos().Distance(dispenser_pos);
          if (dist < nearest_dist) //&& _dispenser_vicinity_box.Intersects(m->BoundingBox()))
          {
            _item_en = en;
            DispenserCommonPtr->_dispenser_filled = true;
            _item_en_found = true;
            nearest_dist = dist;

            /* Link link(ecm.EntityByComponents(
              components::ParentEntity(en),
              components::Name("body"),
              components::Link()));

            std::cout << "Name: " << *link.Name(ecm) << std::endl;
            std::cout << "Pose: " << link.WorldPose(ecm)->Pos() << std::endl;
            */
          }
        }
        return true;
      });

    if (!DispenserCommonPtr->_dispenser_filled)
    {
      RCLCPP_WARN(DispenserCommonPtr->logger(),
        "Could not find dispenser item model within 1 meter, "
        "this dispenser will not be operational");
    } else {
      RCLCPP_INFO(DispenserCommonPtr->logger(),
        "Found dispenser item: [%s]", ecm.Component<components::Name>(_item_en)->Data().c_str());
    }
}

void TeleportDispenserPlugin::create_dispenser_bounding_box(EntityComponentManager& ecm){
  const auto dispenser_pos =  ecm.Component<components::Pose>(_dispenser)->Data().Pos();
  ignition::math::Vector3d corner_1(dispenser_pos.X() - 0.05, dispenser_pos.Y() - 0.05, dispenser_pos.Z() - 0.05);
  ignition::math::Vector3d corner_2(dispenser_pos.X() + 0.05, dispenser_pos.Y() + 0.05, dispenser_pos.Z() + 0.05);
  _dispenser_vicinity_box = ignition::math::AxisAlignedBox(corner_1, corner_2);
  auto aabb = ecm.Component<components::AxisAlignedBox>(_dispenser);
  if (!aabb) {
    ecm.CreateComponent(_dispenser, components::AxisAlignedBox(_dispenser_vicinity_box));
  } else {
    ecm.Component<components::AxisAlignedBox>(_dispenser)->Data() = _dispenser_vicinity_box;
  }
}

void TeleportDispenserPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>&,
  EntityComponentManager& ecm, EventManager&)
{
  char const** argv = NULL;
  if (!rclcpp::is_initialized())
    rclcpp::init(0, argv);

  _dispenser = entity;
  DispenserCommonPtr->_guid = ecm.Component<components::Name>(_dispenser)->Data();
  std::string plugin_name("plugin_" + DispenserCommonPtr->_guid);
  ignwarn << "Initializing plugin with name " << plugin_name << std::endl;

  _ros_node = std::make_shared<rclcpp::Node>(plugin_name);
  DispenserCommonPtr->init_ros_node(_ros_node);
  RCLCPP_INFO(DispenserCommonPtr->_ros_node->get_logger(), "Started node...");
  
  create_dispenser_bounding_box(ecm);

  DispenserCommonPtr->_current_state.guid = DispenserCommonPtr->_guid;
  DispenserCommonPtr->_current_state.mode = DispenserState::IDLE;

  _load_complete = true;
}

void TeleportDispenserPlugin::PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm)
{
  DispenserCommonPtr->_sim_time = (std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count());
  // TODO parallel thread executor?
  rclcpp::spin_some(DispenserCommonPtr->_ros_node);
  if (!_load_complete) {
    return;
  }

  // Only fill dispenser on very first PreUpdate() call. Do it here and not in Configure() so that all models have loaded
  if(!DispenserCommonPtr->_dispenser_filled && !tried_fill_dispenser){
    fill_dispenser(ecm);
    tried_fill_dispenser = true;
  }

  if(DispenserCommonPtr->_dispense){ // Only dispenses max once per call to PreUpdate()
    DispenserCommonPtr->send_dispenser_response(DispenserResult::ACKNOWLEDGED);

    if(DispenserCommonPtr->_dispenser_filled){
      RCLCPP_INFO(DispenserCommonPtr->logger(), "Dispensing item");
      dispense_on_nearest_robot(ecm, DispenserCommonPtr->latest.transporter_type);
      //rclcpp::sleep_for(std::chrono::seconds(5));
      DispenserCommonPtr->send_dispenser_response(DispenserResult::SUCCESS);
      RCLCPP_INFO(DispenserCommonPtr->logger(), "Success");
    } else {
      RCLCPP_WARN(DispenserCommonPtr->logger(),
          "No item to dispense: [%s]", DispenserCommonPtr->latest.request_guid);
        DispenserCommonPtr->send_dispenser_response(DispenserResult::FAILED);
    }
    DispenserCommonPtr->_dispense= false;
  }

  if (DispenserCommonPtr->_sim_time - DispenserCommonPtr->_last_pub_time >= 2.0)
  {
    DispenserCommonPtr->_last_pub_time = DispenserCommonPtr->_sim_time;
    const auto now = DispenserCommonPtr->simulation_now(DispenserCommonPtr->_sim_time);

    DispenserCommonPtr->_current_state.time = now;
    DispenserCommonPtr->_current_state.mode = DispenserState::IDLE;
    DispenserCommonPtr->_state_pub->publish(DispenserCommonPtr->_current_state);


    if(_item_en_found
      && ecm.Component<components::AxisAlignedBox>(_dispenser)->Data().Contains(
        ecm.Component<components::Pose>(_item_en)->Data().Pos())) {
      DispenserCommonPtr->_dispenser_filled = true;
    }
  }
}

IGNITION_ADD_PLUGIN(
  TeleportDispenserPlugin,
  System,
  TeleportDispenserPlugin::ISystemConfigure,
  TeleportDispenserPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(TeleportDispenserPlugin, "teleport_dispenser")

} // namespace rmf_ignition_plugins

