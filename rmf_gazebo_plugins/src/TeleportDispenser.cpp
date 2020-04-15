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

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <ignition/math/Vector3.hh>

#include "utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>

namespace rmf_gazebo_plugins {

class TeleportDispenserPlugin : public gazebo::ModelPlugin
{

public:

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  using Pose3d = ignition::math::Pose3d;

  std::string _dispenser_guid;
  double _last_pub_time = 0.0;
  bool _object_dispensed = false;
  bool _load_complete = false;

  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _dispenser_model;
  gazebo::physics::ModelPtr _dispenser_item_model;
  gazebo::physics::WorldPtr _world;

  gazebo_ros::Node::SharedPtr _node;
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;

  std::unordered_map<std::string, FleetState::UniquePtr> _fleet_states;

  std::unordered_set<std::string> _dispenser_request_guids;

  DispenserState _current_dispenser_state;

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
  {
    _dispenser_model = _parent;
    _world = _dispenser_model->GetWorld();

    // find the dispenser item model, maximum distance 1 meter
    const auto dispenser_bb = _dispenser_model->BoundingBox();
    auto model_list = _world->Models();

    double nearest_model_dist;
    std::string nearest_model_name;
    find_vertical_nearest_model_name(nearest_model_dist, nearest_model_name);
    _dispenser_item_model = _world->ModelByName(nearest_model_name);

    if (!_dispenser_item_model)
    {
      RCLCPP_WARN(_node->get_logger(), 
          "Could not find dispenser item model within 1 meter, "
          "this dispenser will not be operational");
      return;
    }

    RCLCPP_INFO(_node->get_logger(),
        "Found dispenser item: [%s]", nearest_model_name.c_str());

    _node = gazebo_ros::Node::Get(_sdf);
    RCLCPP_INFO(_node->get_logger(), "Started TeleportDispenserPlugin node...");

    _dispenser_guid = _dispenser_model->GetName();

    _fleet_state_sub = _node->create_subscription<FleetState>(
        "/fleet_states",
        rclcpp::SystemDefaultsQoS(),
        [&](FleetState::UniquePtr msg)
        {
          fleet_state_cb(std::move(msg));
        });

    _state_pub = _node->create_publisher<DispenserState>(
        "/dispenser_states", 10);

    _request_sub = _node->create_subscription<DispenserRequest>(
        "/dispenser_requests",
        rclcpp::SystemDefaultsQoS(),
        [&](DispenserRequest::UniquePtr msg)
        {
          dispenser_request_cb(std::move(msg));
        });

    _result_pub = _node->create_publisher<DispenserResult>(
        "/dispenser_results", 10);
    
    _current_dispenser_state.guid = _dispenser_guid;
    _current_dispenser_state.mode = DispenserState::IDLE;

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&TeleportDispenserPlugin::on_update, this));
    _load_complete = true;
  }

  rclcpp::Time simulation_now()
  {
    const double t = _dispenser_model->GetWorld()->SimTime().Double();
    const int32_t t_sec = static_cast<int32_t>(t);
    const uint32_t t_nsec =
        static_cast<uint32_t>((t-static_cast<double>(t_sec)) * 1e9);
    return rclcpp::Time{t_sec, t_nsec, RCL_ROS_TIME};
  }

  // This function is a variation of GetNearestEntityBelow from 
  // gazebo/physics/Entity.cc
  // provides the absolute distance from the center of the model
  // checks for both top and bottom
  void find_vertical_nearest_model_name(
      double& distance, std::string& model_name)
  {
    _world->Physics()->InitForThread();

    using namespace gazebo::physics;
    RayShapePtr ray_shape = boost::dynamic_pointer_cast<RayShape>(
        _world->Physics()->CreateShape("ray", CollisionPtr()));

    auto box = _dispenser_model->BoundingBox();
    ignition::math::Vector3d start;
    ignition::math::Vector3d end;
    
    // checks top
    double top_distance;
    std::string top_model_name;
    start.Z(box.Center().Z());
    end.Z(box.Center().Z() + 1.0);
    ray_shape->SetPoints(start, end);
    ray_shape->GetIntersection(top_distance, top_model_name);

    // checks bottom
    double bottom_distance;
    std::string bottom_model_name;
    start.Z(box.Center().Z());
    end.Z(box.Center().Z() - 1.0);
    ray_shape->SetPoints(start, end);
    ray_shape->GetIntersection(bottom_distance, bottom_model_name);

    model_name = 
        top_distance > bottom_distance? top_model_name : bottom_model_name;
    distance = std::min(top_distance, bottom_distance);
  }

  bool find_nearest_model_name(
      const std::vector<gazebo::physics::ModelPtr>& models,
      std::string& nearest_model_name)
  {
    double nearest_dist = 1e6;
    bool found = false;

    for (const auto& m : models)
    {
      if (!m)
        continue;
      
      double dist =
          m->WorldPose().Pos().Distance(_dispenser_model->WorldPose().Pos());
      if (dist < nearest_dist)
      {
        nearest_dist = dist;
        nearest_model_name = m->GetName();
        found = true;
      }
    }
    return found;
  }

  bool find_nearest_model_name(
      const std::vector<gazebo::physics::ModelPtr>& models,
      double maximum_distance, 
      std::string& nearest_model_name)
  {
    double nearest_dist = maximum_distance;
    bool found = false;

    for (const auto& m : models)
    {
      if (!m)
        continue;
      
      double dist =
          m->WorldPose().Pos().Distance(_dispenser_model->WorldPose().Pos());
      if (dist < nearest_dist)
      {
        nearest_dist = dist;
        nearest_model_name = m->GetName();
        found = true;
      }
    }
    return found;
  }

  void dispense_on_nearest_robot(const std::string& fleet_name)
  {
    if (!_dispenser_item_model)
      return;

    auto fleet_state_it = _fleet_states.find(fleet_name);
    if (fleet_state_it == _fleet_states.end())
    {
      RCLCPP_WARN(_node->get_logger(),
          "No such fleet: [%s]", fleet_name.c_str());
      return;
    }

    std::vector<gazebo::physics::ModelPtr> robot_models;
    for (const auto& rs : fleet_state_it->second->robots)
    {
      auto r_model = _world->ModelByName(rs.name);
      if (r_model)
        robot_models.push_back(r_model);
    }

    std::string nearest_robot_model_name;
    if (!find_nearest_model_name(robot_models, nearest_robot_model_name))
    {
      RCLCPP_WARN(_node->get_logger(),
          "No near robots of fleet [%s] found.", fleet_name.c_str());
      return;
    }
    _dispenser_item_model->PlaceOnEntity(nearest_robot_model_name);
  }

  void fleet_state_cb(FleetState::UniquePtr msg)
  {
    _fleet_states[msg->name] = std::move(msg);
  }

  void dispenser_request_cb(DispenserRequest::UniquePtr msg)
  {
    // TODO: the message field should use fleet name instead
    auto transporter_type = msg->transporter_type;
    auto request_guid = msg->request_guid;
    
    DispenserResult response;
    response.request_guid = msg->request_guid;

    if (_dispenser_guid == msg->target_guid && !_object_dispensed)
    {
      if (!_dispenser_request_guids.insert(request_guid).second)
        return;

      response.time = simulation_now();
      response.source_guid = _dispenser_guid;
      response.status = DispenserResult::ACKNOWLEDGED;
      _result_pub->publish(response);

      RCLCPP_INFO(_node->get_logger(), "Dispensing item");
      dispense_on_nearest_robot(transporter_type);
      rclcpp::sleep_for(std::chrono::seconds(5));
      _object_dispensed = true;

      response.time = simulation_now();
      response.status = DispenserResult::SUCCESS;
      _result_pub->publish(response);
    }
  }

  void on_update()
  {
    if (!_load_complete)
      return;

    const double t = _world->SimTime().Double();
    if (t - _last_pub_time >= 2.0)
    {
      _last_pub_time = t;
      const auto now = simulation_now();

      _current_dispenser_state.time = now;
      _current_dispenser_state.mode =
          _current_dispenser_state.request_guid_queue.empty()?
          DispenserState::IDLE : DispenserState::BUSY;
      _state_pub->publish(_current_dispenser_state);

      if (_object_dispensed &&
          _dispenser_item_model &&
          _dispenser_model->BoundingBox().Intersects(
              _dispenser_item_model->BoundingBox()))
        _object_dispensed = false;
    }
  }

  ~TeleportDispenserPlugin()
  {
    if (_load_complete)
      rclcpp::shutdown();
  }

};

GZ_REGISTER_MODEL_PLUGIN(TeleportDispenserPlugin)

} // namespace rmf_gazebo_plugins
