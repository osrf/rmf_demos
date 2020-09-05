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

#ifndef RMF_PLUGINS_COMMON__DISPENSER_COMMON_HPP
#define RMF_PLUGINS_COMMON__DISPENSER_COMMON_HPP

#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>

namespace rmf_dispenser_common {

class TeleportDispenserCommon
{
public:
  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

  bool dispense = false;
  DispenserRequest latest; // Only store and act on last received request

  std::string guid; // Plugin name

  double last_pub_time = 0.0;
  double sim_time = 0.0;

  bool dispenser_filled = false;

  rclcpp::Node::SharedPtr ros_node;

  std::unordered_map<std::string, FleetState::UniquePtr> fleet_states;
  DispenserState current_state;

  TeleportDispenserCommon();
  rclcpp::Time simulation_now(double t) const;
  void send_dispenser_response(uint8_t status) const;
  void fleet_state_cb(FleetState::UniquePtr msg);
  void dispenser_request_cb(DispenserRequest::UniquePtr msg);
  void init_ros_node(const rclcpp::Node::SharedPtr node);
  void publish_state() const;

private:
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
  rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;
  std::unordered_map<std::string, bool> _past_request_guids;
};

} // namespace rmf_dispenser_common

#endif