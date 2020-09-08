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

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/PoseCmd.hh>
#include <ignition/gazebo/components/Static.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Box.hh>

#include <rclcpp/rclcpp.hpp>

#include <rmf_ingestor_msgs/msg/ingestor_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>

#include <rmf_plugins_common/ingestor_common.hpp>

using namespace ignition::gazebo;
using namespace rmf_ingestor_common;

namespace rmf_ignition_plugins {

class IGNITION_GAZEBO_VISIBLE TeleportIngestorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:

  using IngestorState = rmf_ingestor_msgs::msg::IngestorState;
  using IngestorResult = rmf_ingestor_msgs::msg::IngestorResult;

  TeleportIngestorPlugin();
  ~TeleportIngestorPlugin();
  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override;
  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;

private:
  // Stores params representing state of Ingestor, and handles all message pub/sub
  std::unique_ptr<TeleportIngestorCommon> _ingestor_common;

  Entity _ingestor;
  Entity _ingested_entity; // Item that ingestor may contain

  rclcpp::Node::SharedPtr _ros_node;

  bool find_nearest_model(
    const EntityComponentManager& ecm,
    const std::vector<Entity>& robot_model_entities,
    Entity& robot_entity) const;
  bool get_payload_model(
    const EntityComponentManager& ecm,
    const Entity& robot_entity,
    Entity& payload_entity);
  bool ingest_from_nearest_robot(EntityComponentManager& ecm,
    const std::string& fleet_name);
  void send_ingested_item_home(EntityComponentManager& ecm);
};

bool TeleportIngestorPlugin::find_nearest_model(
  const EntityComponentManager& ecm,
  const std::vector<Entity>& robot_model_entities,
  Entity& robot_entity) const
{
  double nearest_dist = 1e6;
  bool found = false;
  const auto ingestor_pos =
    ecm.Component<components::Pose>(_ingestor)->Data().Pos();

  for (const auto& en : robot_model_entities)
  {
    
    std::string name = ecm.Component<components::Name>(en)->Data();
    if (name == _ingestor_common->_guid)
      continue;

    const auto en_pos = ecm.Component<components::Pose>(en)->Data().Pos();
    const double dist = en_pos.Distance(ingestor_pos);
    if (dist < nearest_dist)
    {
      nearest_dist = dist;
      robot_entity = en;
      found = true;
    }
  }
  return found;
}

bool TeleportIngestorPlugin::get_payload_model(
  const EntityComponentManager& ecm,
  const Entity& robot_entity,
  Entity& payload_entity)
{
  // There might not be a better way to loop through all the models, as we
  // might consider delivering items that were spawned during run time,
  // instead of during launch.
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

bool TeleportIngestorPlugin::ingest_from_nearest_robot(
  EntityComponentManager& ecm, const std::string& fleet_name)
{
  const auto fleet_state_it = _ingestor_common->fleet_states.find(fleet_name);
  if (fleet_state_it == _ingestor_common->fleet_states.end())
  {
    RCLCPP_WARN(_ingestor_common->ros_node->get_logger(),
      "No such fleet: [%s]", fleet_name.c_str());
    return false;
  }

  std::vector<Entity> robot_model_list;
  for (const auto& rs : fleet_state_it->second->robots)
  {
    std::vector<Entity> entities =
      ecm.EntitiesByComponents(components::Name(rs.name),
        components::Model(), components::Static(false));
    robot_model_list.insert(robot_model_list.end(),
      entities.begin(), entities.end());
  }

  Entity robot_model;
  if (!find_nearest_model(ecm, robot_model_list, robot_model))
  {
    RCLCPP_WARN(_ingestor_common->ros_node->get_logger(),
      "No nearby robots of fleet [%s] found.", fleet_name.c_str());
    return false;
  }

  if (!get_payload_model(ecm, robot_model, _ingested_entity))
  {
    RCLCPP_WARN(_ingestor_common->ros_node->get_logger(),
      "No delivery item found on the robot: [%s]",
      Model(robot_model).Name(ecm));
    return false;
  }

  auto cmd = ecm.Component<components::WorldPoseCmd>(_ingested_entity);
  if (!cmd)
  {
    ecm.CreateComponent(_ingested_entity,
      components::WorldPoseCmd(ignition::math::Pose3<double>()));
  }
  auto new_pose = ecm.Component<components::Pose>(_ingestor)->Data();
  ecm.Component<components::WorldPoseCmd>(_ingested_entity)->Data() = new_pose;
  _ingestor_common->ingestor_filled = true;
  return true;
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
        it->second;
    }
    _ingestor_common->ingestor_filled = false;
  }
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
  if (!rclcpp::is_initialized())
    rclcpp::init(0, argv);

  _ingestor = entity;
  _ingestor_common->_guid = ecm.Component<components::Name>(_ingestor)->Data();
  ignwarn << "Initializing plugin with name " << _ingestor_common->_guid
          << std::endl;
  _ros_node = std::make_shared<rclcpp::Node>(_ingestor_common->_guid);
  _ingestor_common->init_ros_node(_ros_node);
  RCLCPP_INFO(_ingestor_common->ros_node->get_logger(), "Started node...");

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
        _ingestor_common->non_static_models_init_poses[name->Data()] = pose->Data();
      }
      return true;
    });

  _ingestor_common->current_state.guid = _ingestor_common->_guid;
  _ingestor_common->current_state.mode = IngestorState::IDLE;
}

void TeleportIngestorPlugin::PreUpdate(const UpdateInfo& info,
  EntityComponentManager& ecm)
{
  _ingestor_common->sim_time =
    std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count();

  // TODO parallel thread executor?
  rclcpp::spin_some(_ingestor_common->ros_node);

  // Only ingests max once per call to PreUpdate(), using request stored in _ingestor_common->_latest
  if (_ingestor_common->ingest)
  {
    _ingestor_common->send_ingestor_response(IngestorResult::ACKNOWLEDGED);

    if (!_ingestor_common->ingestor_filled)
    {
      RCLCPP_INFO(_ingestor_common->ros_node->get_logger(), "Ingesting item");
      bool res = ingest_from_nearest_robot(ecm, _ingestor_common->latest.transporter_type);
      if (res)
      {
        _ingestor_common->send_ingestor_response(IngestorResult::SUCCESS);
        _ingestor_common->last_ingested_time = _ingestor_common->sim_time;
        RCLCPP_INFO(_ingestor_common->ros_node->get_logger(), "Success");
      }
      else
      {
        _ingestor_common->send_ingestor_response(IngestorResult::FAILED);
        RCLCPP_WARN(_ingestor_common->ros_node->get_logger(), "Unable to dispense item");
      }
    }
    else
    {
      RCLCPP_WARN(_ingestor_common->ros_node->get_logger(),
        "No item to ingest: [%s]", _ingestor_common->latest.request_guid);
      _ingestor_common->send_ingestor_response(IngestorResult::FAILED);
    }
    _ingestor_common->ingest = false;
  }

  constexpr double interval = 2.0;
  if (_ingestor_common->sim_time - _ingestor_common->last_pub_time >= interval)
  {
    _ingestor_common->last_pub_time = _ingestor_common->sim_time;
    const auto now = _ingestor_common->simulation_now(
      _ingestor_common->sim_time);

    _ingestor_common->current_state.time = now;
    _ingestor_common->current_state.mode = IngestorState::IDLE;
    _ingestor_common->publish_state();
  }

  // Periodically try to teleport ingested item back to original location
  if (_ingestor_common->sim_time - _ingestor_common->last_ingested_time >=
    5.0 && _ingestor_common->ingestor_filled)
  {
    send_ingested_item_home(ecm);
  }
}

IGNITION_ADD_PLUGIN(
  TeleportIngestorPlugin,
  System,
  TeleportIngestorPlugin::ISystemConfigure,
  TeleportIngestorPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(TeleportIngestorPlugin, "teleport_ingestor")

} // namespace rmf_ignition_plugins

