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
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

/*#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>*/

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_result.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>


namespace rmf_dispenser_common {

class TeleportDispenserCommon {
  public:
    using FleetState = rmf_fleet_msgs::msg::FleetState;
    using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
    using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
    using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;

    // Dispense request params
    bool _dispense = false;
    DispenserRequest latest;

    std::string _guid;

    double _last_pub_time = 0.0;
    double _sim_time = 0.0;

    //bool _load_complete = false;
    //bool _item_en_found = false; // True if entity to be dispensed has been determined. Used to locate item in future
    bool _dispenser_filled = false; // True if entity is in the dispenser
    //bool tried_fill_dispenser = false; // Flag set to true if fill_dispenser() has been called

    rclcpp::Node::SharedPtr _ros_node;
    rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
    rclcpp::Publisher<DispenserState>::SharedPtr _state_pub;
    rclcpp::Subscription<DispenserRequest>::SharedPtr _request_sub;
    rclcpp::Publisher<DispenserResult>::SharedPtr _result_pub;
    
    std::unordered_map<std::string, FleetState::UniquePtr> _fleet_states;
    std::unordered_map<std::string, bool> _past_request_guids;
    DispenserState _current_state;

    TeleportDispenserCommon(){
    }

    rclcpp::Logger logger() const
    {
        return rclcpp::get_logger("Teleport_Dispenser");
    }

    rclcpp::Time simulation_now(double t) const
    {
        const int32_t t_sec = static_cast<int32_t>(t);
        const uint32_t t_nsec =
        static_cast<uint32_t>((t-static_cast<double>(t_sec)) * 1e9);
        return rclcpp::Time{t_sec, t_nsec, RCL_ROS_TIME};
    }

    void send_dispenser_response(uint8_t status) const
    {
        DispenserResult response;
        response.time = simulation_now(_sim_time);
        response.request_guid = latest.request_guid;
        response.source_guid = _guid;
        response.status = status;
        _result_pub->publish(response);
        std::cout << "Publishing: " << response.source_guid << std::endl;
    }

    void fleet_state_cb(FleetState::UniquePtr msg)
    {
        _fleet_states[msg->name] = std::move(msg);
    }

    void dispenser_request_cb(DispenserRequest::UniquePtr msg)
    {
        latest = *msg;

        if (_guid == latest.target_guid && _dispenser_filled)
        {
            const auto it = _past_request_guids.find(latest.request_guid);
            if (it != _past_request_guids.end())
            {
            if (it->second)
            {
                RCLCPP_WARN(_ros_node->get_logger(),
                "Request already succeeded: [%s]", latest.request_guid);
                send_dispenser_response(DispenserResult::SUCCESS);
            }
            else
            {
                RCLCPP_WARN(_ros_node->get_logger(),
                "Request already failed: [%s]", latest.request_guid);
                send_dispenser_response(DispenserResult::FAILED);
            }
            return;
            }

            _dispense = true; // mark true to dispense item next time PreUpdate() is called
            // There are currently no cases to publish a FAILED result yet
        }
    }

    void init_ros_node(const rclcpp::Node::SharedPtr node)
    {
        _ros_node = std::move(node);

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
    }

};

} // namespace rmf_dispenser_common

#endif