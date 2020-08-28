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

// TODO remove this
using namespace ignition;
using namespace ignition::gazebo;

namespace rmf_ignition_plugins {

class IGNITION_GAZEBO_VISIBLE TeleportIngestorPlugin
  : public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

private:
  // Ingest request params
  bool _ingest = false;
  DispenserRequest latest;

  // General params
  Entity _ingestor;
  Entity _ingested_entity;
  std::string _guid;
  bool _load_complete = false;
  bool _ingestor_filled = false;

  double _last_pub_time = 0.0;
  double _last_ingested_time = 0.0;
  std::chrono::steady_clock::duration _sim_time;

  rclcpp::Node::SharedPtr _ros_node;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;

  std::unordered_map<std::string, ignition::math::Pose3d>
  _non_static_models_init_poses;
  std::unordered_map<std::string, FleetState::UniquePtr> _fleet_states;
  std::unordered_map<std::string, bool> _past_request_guids;
  DispenserState _current_state;

  rclcpp::Time simulation_now(const double t) const
  {
    const int32_t t_sec = static_cast<int32_t>(t);
    const uint32_t t_nsec =
      static_cast<uint32_t>((t-static_cast<double>(t_sec)) * 1e9);
    return rclcpp::Time{t_sec, t_nsec, RCL_ROS_TIME};
  }

  bool find_nearest_non_static_model(
    const EntityComponentManager& ecm,
    const std::vector<Entity>& robot_model_entities,
    Entity& robot_entity) const
  {
    double nearest_dist = 1e6;
    bool found = false;
    const auto ingestor_pos = ecm.Component<components::Pose>(_ingestor)->Data().Pos();

    for (const auto& en : robot_model_entities)
    {
      bool is_static = ecm.Component<components::Static>(en)->Data();
      std::string name = ecm.Component<components::Name>(en)->Data();

      if (!en || is_static || name == _guid)
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

  bool get_payload_model(
    const EntityComponentManager& ecm,
    const Entity& robot_entity,
    Entity& payload_entity)
  {
    if(ecm.Component<components::AxisAlignedBox>(robot_entity)){
      std::cout << "Bounding box: " << ecm.Component<components::AxisAlignedBox>(robot_entity)->Data() << std::endl;
    }
    //const ignition::math::Box robot_collision_bb = ecm.Component<components::BoundingBox>(robot_entity)->Data(); //robot_model->BoundingBox();
    //ignition::math::Vector3d max_corner = robot_collision_bb.Max(); //

    // create a new bounding box extended slightly in the Z direction
    //max_corner.Z(max_corner.Z() + 0.1);
    //const ignition::math::Box vicinity_box(robot_collision_bb.Min(), max_corner);

    // There might not be a better way to loop through all the models, as we
    // might consider delivering items that were spawned during run time,
    // instead of during launch.
    const auto robot_model_pos = ecm.Component<components::Pose>(robot_entity)->Data().Pos(); //need world pose
    double nearest_dist = 1.0; //what is this number
    bool found = false;

    ecm.Each<components::Model, components::Name, components::Pose,
      components::Static>( //to check Static param
      [&](const Entity& entity,
      const components::Model*,
      const components::Name* name,
      const components::Pose* pose,
      const components::Static* is_static
      ) -> bool
      {
        if (!is_static->Data() && name->Data() != _guid 
        && name->Data() != Model(robot_entity).Name(ecm))
        {
          const double dist = pose->Data().Pos().Distance(robot_model_pos);
          if (dist < nearest_dist) //&& vicinity_box.Intersects(model->BoundingBox()))//
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

  void ingest_from_nearest_robot(EntityComponentManager& ecm, const std::string& fleet_name)
  {
    const auto fleet_state_it = _fleet_states.find(fleet_name);
    if (fleet_state_it == _fleet_states.end())
    {
      RCLCPP_WARN(_ros_node->get_logger(),
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
      RCLCPP_WARN(_ros_node->get_logger(),
        "No nearby robots of fleet [%s] found.", fleet_name.c_str());
      return;
    }

    if (!get_payload_model(ecm, robot_model, _ingested_entity))
    {
      RCLCPP_WARN(_ros_node->get_logger(),
        "No delivery item found on the robot: [%s]",
        Model(robot_model).Name(ecm));
      return;
    }

    auto cmd = ecm.Component<components::WorldPoseCmd>(_ingested_entity);
    if (!cmd) {
      ecm.CreateComponent(_ingested_entity, components::WorldPoseCmd(ignition::math::Pose3<double>()));
    }
    auto new_pose = ecm.Component<components::Pose>(_ingestor)->Data()+ ignition::math::Pose3<double>(0,0,0.5,0,0,0);;
    ecm.Component<components::WorldPoseCmd>(_ingested_entity)->Data() = new_pose;
    _ingestor_filled = true;
  }

  void send_ingested_item_home(EntityComponentManager& ecm)
  {
    if (_ingestor_filled)
    {
      const auto it = _non_static_models_init_poses.find(Model(_ingested_entity).Name(ecm));
      if (it == _non_static_models_init_poses.end()) {
        ecm.RequestRemoveEntity(_ingested_entity);
      } else {
        auto cmd = ecm.Component<components::WorldPoseCmd>(_ingested_entity);
        if (!cmd) {
          ecm.CreateComponent(_ingested_entity, components::WorldPoseCmd(ignition::math::Pose3<double>()));
        }
        ecm.Component<components::WorldPoseCmd>(_ingested_entity)->Data() = it->second;
      }
      _ingestor_filled = false;
    }
  }

  void fleet_state_cb(FleetState::UniquePtr msg)
  {
    _fleet_states[msg->name] = std::move(msg);
  }

  void send_ingestor_response(uint8_t status) const
  {
    DispenserResult response;
    response.time = simulation_now(std::chrono::duration_cast<std::chrono::nanoseconds>(_sim_time).count() * 1e-9); //maybe there's a better way to do this
    response.request_guid = latest.request_guid;
    response.source_guid = _guid;
    response.status = status;
    _result_pub->publish(response);
  }

  void dispenser_request_cb(DispenserRequest::UniquePtr msg)
  {
    latest = *msg;

    if (_guid == latest.target_guid && !_ingestor_filled)
    {
      const auto it = _past_request_guids.find(latest.request_guid);
      if (it != _past_request_guids.end())
      {
        if (it->second)
        {
          RCLCPP_WARN(_ros_node->get_logger(),
            "Request already succeeded: [%s]", latest.request_guid);
          send_ingestor_response(DispenserResult::SUCCESS);
        }
        else
        {
          RCLCPP_WARN(_ros_node->get_logger(),
            "Request already failed: [%s]", latest.request_guid);
          send_ingestor_response(DispenserResult::FAILED);
        }
        return;
      }

      _ingest = true; // mark true to ingest item next time PreUpdate() is called
      // There are currently no cases to publish a FAILED result yet
    }
  }

public:

  void Configure(const Entity& entity,
    const std::shared_ptr<const sdf::Element>&,
    EntityComponentManager& ecm, EventManager&) override
  {
    char const** argv = NULL;
    if (!rclcpp::is_initialized())
      rclcpp::init(0, argv);

    _ingestor = entity;
    _guid = ecm.Component<components::Name>(_ingestor)->Data();
    std::string plugin_name("plugin_" + _guid);
    ignwarn << "Initializing plugin with name " << plugin_name << std::endl;
    _ros_node = std::make_shared<rclcpp::Node>(plugin_name);
    RCLCPP_INFO(_ros_node->get_logger(), "Started node...");

    // Keep track of all the non-static models
    ecm.Each<components::Model, components::Name, components::Pose, components::Static>(
      [&](const Entity&,
      const components::Model*,
      const components::Name* name,
      const components::Pose* pose,
      const components::Static* is_static
      ) -> bool
      {
        if (!is_static->Data() && name->Data() != _guid)
        {
          _non_static_models_init_poses[name->Data()] = pose->Data();
        }
        return true;
      });

    _fleet_state_sub = _ros_node->create_subscription<FleetState>(
      "/fleet_states",
      rclcpp::SystemDefaultsQoS(),
      [&](FleetState::UniquePtr msg)
      {
        fleet_state_cb(std::move(msg));
      });

    _state_pub = _ros_node->create_publisher<DispenserState>(
      "/dispenser_states", 10);

    _request_sub = _ros_node->create_subscription<DispenserRequest>(
      "/dispenser_requests",
      rclcpp::SystemDefaultsQoS(),
      [&](DispenserRequest::UniquePtr msg)
      {
        dispenser_request_cb(std::move(msg));
      });

    _result_pub = _ros_node->create_publisher<DispenserResult>(
      "/dispenser_results", 10);

    _current_state.guid = _guid;
    _current_state.mode = DispenserState::IDLE;

    _load_complete = true;
  }

  void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override
  {
    _sim_time = info.simTime;
    double t =
    (std::chrono::duration_cast<std::chrono::nanoseconds>(_sim_time).
    count()) * 1e-9;

    // TODO parallel thread executor?
    rclcpp::spin_some(_ros_node);
    if (!_load_complete) {
      return;
    }

    if(_ingest) { // Only ingests max once per call to PreUpdate()
      send_ingestor_response(DispenserResult::ACKNOWLEDGED);

      RCLCPP_INFO(_ros_node->get_logger(), "Ingesting item");
      ingest_from_nearest_robot(ecm, latest.transporter_type);

      send_ingestor_response(DispenserResult::SUCCESS);

      //rclcpp::sleep_for(std::chrono::seconds(10));
      //send_ingested_item_home(ecm);
      _last_ingested_time = t;
      _ingest = false;
    }

    if (t - _last_pub_time >= 2.0)
    {
      _last_pub_time = t;
      const auto now = simulation_now(t);

      _current_state.time = now;
      _current_state.mode = DispenserState::IDLE;
      _state_pub->publish(_current_state);
    }

    if(t - _last_ingested_time >= 5.0 && _ingestor_filled){
      send_ingested_item_home(ecm); 
    }
  }
};

IGNITION_ADD_PLUGIN(
  TeleportIngestorPlugin,
  System,
  TeleportIngestorPlugin::ISystemConfigure,
  TeleportIngestorPlugin::ISystemPreUpdate)

// TODO would prefer namespaced
IGNITION_ADD_PLUGIN_ALIAS(TeleportIngestorPlugin, "teleport_ingestor")

} // namespace rmf_ignition_plugins

