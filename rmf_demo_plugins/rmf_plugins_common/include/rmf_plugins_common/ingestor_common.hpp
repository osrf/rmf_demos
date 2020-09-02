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

#ifndef RMF_PLUGINS_COMMON__INGESTOR_COMMON_HPP
#define RMF_PLUGINS_COMMON__INGESTOR_COMMON_HPP

#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

#include <ignition/math/Pose3.hh>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>

namespace rmf_ingestor_common {

class TeleportIngestorCommon {
  public:
    using FleetState = rmf_fleet_msgs::msg::FleetState;
    using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
    using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
    using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

    // Ingest request params
    bool _ingest = false;
    DispenserRequest latest;

    // General params
    std::string _guid;
    bool _load_complete = false;
    bool _ingestor_filled = false;

    double _last_pub_time = 0.0;
    double _last_ingested_time = 0.0;
    double _sim_time = 0.0;

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

    TeleportIngestorCommon();
    rclcpp::Time simulation_now(double t) const;
    void send_ingestor_response(uint8_t status) const;
    void fleet_state_cb(FleetState::UniquePtr msg);
    void dispenser_request_cb(DispenserRequest::UniquePtr msg);
    void init_ros_node(const rclcpp::Node::SharedPtr node);
};

} // namespace rmf_ingestor_common

#endif
