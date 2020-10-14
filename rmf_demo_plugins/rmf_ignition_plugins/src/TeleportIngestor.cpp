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
#include <functional>
#include <memory>
#include <string>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Static.hh>
#include <ignition/gazebo/components/LinearVelocityCmd.hh>
#include <ignition/gazebo/components/AngularVelocityCmd.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <rclcpp/rclcpp.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_plugins_common/ingestor_common.hpp>

using namespace ignition::gazebo;
using namespace rmf_ingestor_common;
using namespace rmf_plugins_utils;

namespace rmf_ignition_plugins {

class IGNITION_GAZEBO_VISIBLE TeleportIngestorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStateIt =
    std::unordered_map<std::string, FleetState::UniquePtr>::iterator;

  TeleportIngestorPlugin();
  ~TeleportIngestorPlugin();
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  // Stores params representing state of Ingestor, and handles all message pub/sub
  std::unique_ptr<TeleportIngestorCommon> _ingestor_common;

  bool _non_static_models_filled = false;
  Entity _ingestor;
  Entity _ingested_entity; // Item that ingestor may contain

  rclcpp::Node::SharedPtr _ros_node;
  ignition::transport::Node _ign_node;
  ignition::transport::Node::Publisher _item_ingested_pub;

  SimEntity find_nearest_model(const EntityComponentManager& ecm,
    const std::vector<SimEntity>& robot_model_entities,
    bool& found) const;
  bool get_payload_model(const EntityComponentManager& ecm,
    const SimEntity& robot_sim_entity,
    Entity& payload_entity);
  void fill_robot_list(EntityComponentManager& ecm,
    FleetStateIt fleet_state_it, std::vector<SimEntity>& robot_list);
  void transport_model(EntityComponentManager& ecm);
  void send_ingested_item_home(EntityComponentManager& ecm);
  void init_non_static_models_poses(EntityComponentManager& ecm);
};

SimEntity TeleportIngestorPlugin::find_nearest_model(
  const EntityComponentManager& ecm,
  const std::vector<SimEntity>& entities,
  bool& found) const
{
  double nearest_dist = 1e6;
  SimEntity robot_entity(0);
  const auto ingestor_pos =
    ecm.Component<components::Pose>(_ingestor)->Data().Pos();

  for (const auto& sim_obj : entities)
  {
    // If `models` has been generated with `fill_robot_list`, it is
    // guaranteed to have a valid entity field
    Entity en = sim_obj.get_entity();
    std::string name = ecm.Component<components::Name>(en)->Data();
    if (name == _ingestor_common->_guid)
      continue;

    const auto en_pos = ecm.Component<components::Pose>(en)->Data().Pos();
    const double dist = en_pos.Distance(ingestor_pos);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      robot_entity = sim_obj;
      found = true;
    }
  }
  return robot_entity;
}

// Identifies item to ingest and assigns it to `payload_entity`
bool TeleportIngestorPlugin::get_payload_model(
  const EntityComponentManager& ecm,
  const SimEntity& robot_sim_entity,
  Entity& payload_entity)
{
  // There might not be a better way to loop through all the models, as we
  // might consider delivering items that were spawned during run time,
  // instead of during launch.
  const Entity robot_entity = robot_sim_entity.get_entity();
  const auto robot_model_pos =
    ecm.Component<components::Pose>(robot_entity)->Data().Pos();
  double nearest_dist = 1.0;
  bool found = false;

  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity& entity,
    const components::Model*,
    const components::Name* name,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      if (!is_static->Data() && name->Data() != _ingestor_common->_guid
      && name->Data() != Model(robot_entity).Name(ecm))
      {
        const double dist = pose->Data().Pos().Distance(robot_model_pos);
        if (dist < nearest_dist)
        {
          payload_entity = entity;
          nearest_dist = dist;
          found = true;
        }
      }
      return true;
    });

  return found;
}

void TeleportIngestorPlugin::fill_robot_list(EntityComponentManager& ecm,
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

// Moves the identified item to ingest from its current position to the ingestor
void TeleportIngestorPlugin::transport_model(EntityComponentManager& ecm)
{
  // Ingestor assumes control of entity. Set its pose and cancel any pre-existing velocity cmds
  auto cmd = ecm.Component<components::WorldPoseCmd>(_ingested_entity);
  if (!cmd)
  {
    ecm.CreateComponent(_ingested_entity,
      components::WorldPoseCmd(ignition::math::Pose3<double>()));
  }
  auto new_pose = ecm.Component<components::Pose>(_ingestor)->Data();
  ecm.Component<components::WorldPoseCmd>(_ingested_entity)->Data() = new_pose;

  if (ecm.EntityHasComponentType(_ingested_entity,
    components::LinearVelocityCmd().TypeId()))
  {
    ecm.Component<components::LinearVelocityCmd>(_ingested_entity)->Data() = {
      0, 0, 0};
  }
  if (ecm.EntityHasComponentType(_ingested_entity,
    components::AngularVelocityCmd().TypeId()))
  {
    ecm.Component<components::AngularVelocityCmd>(_ingested_entity)->Data() = {
      0, 0, 0};
  }

  // For Ignition slotcar plugin to know when an item has been ingested from it
  // Necessary for TPE Plugin
  ignition::msgs::Entity ingest_msg;
  ingest_msg.set_id(_ingested_entity); // Implicit conversion to protobuf::uint64
  _item_ingested_pub.Publish(ingest_msg);
}

void TeleportIngestorPlugin::send_ingested_item_home(
  EntityComponentManager& ecm)
{
  if (_ingestor_common->ingestor_filled)
  {
    const auto it =
      _ingestor_common->non_static_models_init_poses.find(
      Model(_ingested_entity).Name(ecm));
    if (it == _ingestor_common->non_static_models_init_poses.end())
    {
      ecm.RequestRemoveEntity(_ingested_entity);
    }
    else
    {
      auto cmd = ecm.Component<components::WorldPoseCmd>(_ingested_entity);
      if (!cmd)
      {
        ecm.CreateComponent(_ingested_entity,
          components::WorldPoseCmd(ignition::math::Pose3<double>()));
      }
      ecm.Component<components::WorldPoseCmd>(_ingested_entity)->Data() =
        convert_to_pose<ignition::math::v6::Pose3d>(it->second);
    }
    _ingestor_common->ingestor_filled = false;
  }
}

void TeleportIngestorPlugin::init_non_static_models_poses(
  EntityComponentManager& ecm)
{
  // Keep track of all the non-static models
  ecm.Each<components::Model, components::Name, components::Pose,
    components::Static>(
    [&](const Entity&,
    const components::Model*,
    const components::Name* name,
    const components::Pose* pose,
    const components::Static* is_static
    ) -> bool
    {
      if (!is_static->Data() && name->Data() != _ingestor_common->_guid)
      {
        _ingestor_common->non_static_models_init_poses[name->Data()] = convert_pose(
          pose->Data());
      }
      return true;
    });
}

TeleportIngestorPlugin::TeleportIngestorPlugin()
: _ingestor_common(std::make_unique<TeleportIngestorCommon>())
{
}

TeleportIngestorPlugin::~TeleportIngestorPlugin()
{
  rclcpp::shutdown();
}

void TeleportIngestorPlugin::Configure(const Entity& entity,
  const std::shared_ptr<const sdf::Element>&,
  EntityComponentManager& ecm, EventManager&)
{
  char const** argv = NULL;
  if (!rclcpp::ok())
    rclcpp::init(0, argv);

  _ingestor = entity;
  _ingestor_common->_guid = ecm.Component<components::Name>(_ingestor)->Data();
  ignwarn << "Initializing plugin with name " << _ingestor_common->_guid
          << std::endl;
  _ros_node = std::make_shared<rclcpp::Node>(_ingestor_common->_guid);
  _ingestor_common->init_ros_node(_ros_node);
  RCLCPP_INFO(_ingestor_common->ros_node->get_logger(),
    "Started TeleportIngestorPlugin node...");

  // Needed for TPE plugin, so that subscriber knows when to stop moving a payload that
  // has been ingested from it
  _item_ingested_pub = _ign_node.Advertise<ignition::msgs::Entity>(
    "/item_ingested");
  if (!_item_ingested_pub)
  {
    ignwarn << "Error advertising topic [/item_ingested]" << std::endl;
  }
}

void TeleportIngestorPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  _ingestor_common->sim_time =
    std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count();

  if (!_non_static_models_filled)
  {
    // Initialize here and not in configure to allow all models to load
    init_non_static_models_poses(ecm);
    _non_static_models_filled = true;
  }

  // TODO parallel thread executor?
  rclcpp::spin_some(_ingestor_common->ros_node);

  std::function<void(void)> send_ingested_item_home_cb =
    std::bind(&TeleportIngestorPlugin::send_ingested_item_home,
      this, std::ref(ecm));

  std::function<void(FleetStateIt,
    std::vector<SimEntity>&)> fill_robot_list_cb =
    std::bind(&TeleportIngestorPlugin::fill_robot_list, this,
      std::ref(ecm), std::placeholders::_1, std::placeholders::_2);

  std::function<SimEntity(const std::vector<SimEntity>&,
    bool&)> find_nearest_model_cb =
    std::bind(&TeleportIngestorPlugin::find_nearest_model, this,
      std::ref(ecm), std::placeholders::_1, std::placeholders::_2);

  std::function<bool(const SimEntity&)> get_payload_model_cb =
    std::bind(&TeleportIngestorPlugin::get_payload_model, this,
      std::ref(ecm), std::placeholders::_1, std::ref(_ingested_entity));

  std::function<void()> transport_model_cb =
    std::bind(&TeleportIngestorPlugin::transport_model, this,
      std::ref(ecm));

  _ingestor_common->on_update(fill_robot_list_cb, find_nearest_model_cb,
    get_payload_model_cb, transport_model_cb, send_ingested_item_home_cb);
}

IGNITION_ADD_PLUGIN(
  TeleportIngestorPlugin,
  System,
  TeleportIngestorPlugin::ISystemConfigure,
  TeleportIngestorPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(TeleportIngestorPlugin, "teleport_ingestor")

} // namespace rmf_ignition_plugins

