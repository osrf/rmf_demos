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

#include <rmf_plugins_common/dispenser_common.hpp>

using namespace rmf_dispenser_common;

namespace rmf_gazebo_plugins {

class TeleportDispenserPlugin : public gazebo::ModelPlugin
{

public:

  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

private:
  std::unique_ptr<TeleportDispenserCommon> DispenserCommonPtr;

  bool _load_complete = false;

  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  gazebo::physics::ModelPtr _item_model;
  gazebo::physics::WorldPtr _world;

  #if GAZEBO_MAJOR_VERSION <= 9
  ignition::math::Box _dispenser_vicinity_box;
  #else
  ignition::math::AxisAlignedBox _dispenser_vicinity_box;
  #endif

  bool find_nearest_non_static_model_name(
    const std::vector<gazebo::physics::ModelPtr>& models,
    std::string& nearest_model_name) const
  {
    double nearest_dist = 1e6;
    bool found = false;

    for (const auto& m : models)
    {
      if (!m || m->IsStatic() || m->GetName() == DispenserCommonPtr->_guid)
        continue;

      const double dist =
        m->WorldPose().Pos().Distance(_model->WorldPose().Pos());
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
    if (!_item_model)
      return;

    const auto fleet_state_it = DispenserCommonPtr->_fleet_states.find(fleet_name);
    if (fleet_state_it == DispenserCommonPtr->_fleet_states.end())
    {
      RCLCPP_WARN(DispenserCommonPtr->_ros_node->get_logger(),
        "No such fleet: [%s]", fleet_name.c_str());
      return;
    }

    std::vector<gazebo::physics::ModelPtr> robot_models;
    for (const auto& rs : fleet_state_it->second->robots)
    {
      const auto r_model = _world->ModelByName(rs.name);
      if (r_model)
        robot_models.push_back(r_model);
    }

    std::string nearest_robot_model_name;
    if (!find_nearest_non_static_model_name(
        robot_models, nearest_robot_model_name))
    {
      RCLCPP_WARN(DispenserCommonPtr->_ros_node->get_logger(),
        "No near robots of fleet [%s] found.", fleet_name.c_str());
      return;
    }
    _item_model->PlaceOnEntity(nearest_robot_model_name);
    DispenserCommonPtr->_dispenser_filled = false;
  }

  void fill_dispenser(){
    auto model_list = _world->Models();
    double nearest_dist = 1.0;
    const auto dispenser_pos = _model->WorldPose().Pos();
    for (const auto& m : model_list)
    {
      if (!m || m->IsStatic() || m->GetName() == _model->GetName())
        continue;

      const double dist = m->WorldPose().Pos().Distance(dispenser_pos);
      if (dist < nearest_dist &&
        _dispenser_vicinity_box.Intersects(m->BoundingBox()))
      {
        _item_model = m;
        nearest_dist = dist;
        DispenserCommonPtr->_dispenser_filled = true;
      }
    }
  }

  void create_dispenser_bounding_box(){
    // find the dispenser item model, maximum distance 1 meter
    const auto dispenser_pos = _model->WorldPose().Pos();
    ignition::math::Vector3d corner_1 = dispenser_pos;
    corner_1.X(dispenser_pos.X() - 0.05);
    corner_1.Y(dispenser_pos.Y() - 0.05);
    corner_1.Z(dispenser_pos.Z() - 0.05);
    ignition::math::Vector3d corner_2 = dispenser_pos;
    corner_2.X(dispenser_pos.X() + 0.05);
    corner_2.Y(dispenser_pos.Y() + 0.05);
    corner_2.Z(dispenser_pos.Z() + 0.05);

    #if GAZEBO_MAJOR_VERSION <= 9
    _dispenser_vicinity_box = ignition::math::Box(corner_1, corner_2);
    #else
    _dispenser_vicinity_box =
      ignition::math::AxisAlignedBox(corner_1, corner_2);
    #endif
  }

  void on_update()
  {
    if (!_load_complete)
      return;

    DispenserCommonPtr->_sim_time = _world->SimTime().Double();

    if(DispenserCommonPtr->_dispense){
      if(DispenserCommonPtr->_dispenser_filled){
        DispenserCommonPtr->send_dispenser_response(DispenserResult::ACKNOWLEDGED);

        RCLCPP_INFO(DispenserCommonPtr->_ros_node->get_logger(), "Dispensing item");
        dispense_on_nearest_robot(DispenserCommonPtr->latest.transporter_type);
        //rclcpp::sleep_for(std::chrono::seconds(5));

        DispenserCommonPtr->send_dispenser_response(DispenserResult::SUCCESS);
      } else {
          RCLCPP_WARN(DispenserCommonPtr->_ros_node->get_logger(),
            "No item to dispense: [%s]", DispenserCommonPtr->latest.request_guid);
          DispenserCommonPtr->send_dispenser_response(DispenserResult::FAILED);
      }
      DispenserCommonPtr->_dispense = false;
    }

    const double t = _world->SimTime().Double();
    if (t - DispenserCommonPtr->_last_pub_time >= 2.0)
    {
      DispenserCommonPtr->_last_pub_time = t;
      const auto now = DispenserCommonPtr->simulation_now(t);

      DispenserCommonPtr->_current_state.time = now;
      DispenserCommonPtr->_current_state.mode = DispenserState::IDLE;
      DispenserCommonPtr->_state_pub->publish(DispenserCommonPtr->_current_state);

      if (!DispenserCommonPtr->_dispenser_filled &&
        _item_model &&
        _model->BoundingBox().Intersects(
          _item_model->BoundingBox()))
        DispenserCommonPtr->_dispenser_filled = true;
    }
  }

public:

  TeleportDispenserPlugin()
  : DispenserCommonPtr(std::make_unique<TeleportDispenserCommon>())
  {
  }

  ~TeleportDispenserPlugin()
  {
    if (_load_complete)
      rclcpp::shutdown();
  }

  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override
  {
    _model = _parent;
    _world = _model->GetWorld();
    DispenserCommonPtr->_guid = _model->GetName();

    DispenserCommonPtr->init_ros_node(gazebo_ros::Node::Get(_sdf));
    RCLCPP_INFO(DispenserCommonPtr->_ros_node->get_logger(), "Started TeleportDispenserPlugin node...");

    create_dispenser_bounding_box();
    fill_dispenser();

    if (!_item_model)
    {
      RCLCPP_WARN(DispenserCommonPtr->_ros_node->get_logger(),
        "Could not find dispenser item model within 1 meter, "
        "this dispenser will not be operational");
      return;
    }

    RCLCPP_INFO(DispenserCommonPtr->_ros_node->get_logger(),
      "Found dispenser item: [%s]", _item_model->GetName().c_str());

    DispenserCommonPtr->_current_state.guid = DispenserCommonPtr->_guid;
    DispenserCommonPtr->_current_state.mode = DispenserState::IDLE;

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&TeleportDispenserPlugin::on_update, this));
    _load_complete = true;
  }
};

} // namespace rmf_gazebo_plugins

GZ_REGISTER_MODEL_PLUGIN(rmf_gazebo_plugins::TeleportDispenserPlugin)
