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


namespace rmf_ignition_plugins {

class TeleportDispenserCommon {
  public:
      rclcpp::Time simulation_now(double t) const
      {
        const int32_t t_sec = static_cast<int32_t>(t);
        const uint32_t t_nsec =
        static_cast<uint32_t>((t-static_cast<double>(t_sec)) * 1e9);
        return rclcpp::Time{t_sec, t_nsec, RCL_ROS_TIME};
      }

      /*bool find_nearest_non_static_model_name(std::vector<bool> statics, std::vector<std::string> names, std::vector<ignition::math::Pose3d> poses,
        ignition::math::Pose3d dispenser_pose, std::string& dispenser_name, int& idx){
            double nearest_dist = 1e6;
            bool found = false;

            for(int i = 0; i < statics.size(); ++i){ //assume all vectors have same size
                if(statics[i] || names[i] == dispenser_name){
                    continue;
                }

                const double dist = dispenser_pose.Pos().Distance(poses[i].Pos()):
                if (dist < nearest_dist)
                {
                    nearest_dist = dist;
                    idx = i;
                    found = true;
                }
            }
            return found;
        }*/

};

} // namespace rmf_ignition_plugins

#endif