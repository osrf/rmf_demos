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

#ifndef RMF_PLUGINS_COMMON__READONLY_COMMON_HPP
#define RMF_PLUGINS_COMMON__READONLY_COMMON_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <string>

#include <Eigen/Geometry>

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <building_map_msgs/msg/building_map.hpp>
#include <building_map_msgs/msg/level.hpp>
#include <building_map_msgs/msg/graph.hpp>

namespace rmf_readonly_common {

class ReadonlyCommon
{
public:

  using BuildingMap = building_map_msgs::msg::BuildingMap;
  using Level = building_map_msgs::msg::Level;
  using Graph = building_map_msgs::msg::Graph;
  using Location = rmf_fleet_msgs::msg::Location;
  using Path = std::vector<Location>;

  rclcpp::Node::SharedPtr ros_node;

  void set_name(const std::string& name);
  std::string get_name() const;
  rclcpp::Logger logger();
  template<typename SdfPtrT>
  void read_sdf(SdfPtrT& sdf);
  void init(rclcpp::Node::SharedPtr node);
  void on_update(Eigen::Isometry3d& pose, double sim_time);

private:
  std::string _name = "caddy"; // Placeholder

  // Updated in each on_update() call
  Eigen::Isometry3d _pose;
  double _sim_time = 0.0;

  rclcpp::Publisher<rmf_fleet_msgs::msg::RobotState>::SharedPtr _robot_state_pub;
  rclcpp::Subscription<BuildingMap>::SharedPtr _building_map_sub;

  rmf_fleet_msgs::msg::RobotState _robot_state_msg;
  rmf_fleet_msgs::msg::RobotMode _current_mode;

  bool _found_level = false;
  bool _found_graph = false;
  bool _initialized_graph = false;
  bool _initialized_start = false;

  // Store cache of BuildingMap
  Path _path;
  Level _level;
  Graph _graph;

  std::string _level_name = "L1";
  std::size_t _nav_graph_index = 1;
  std::string _start_wp_name = "caddy";
  // The number of waypoints to add to the predicted path. Minimum is 1.
  std::size_t _lookahead = 1;
  std::size_t _start_wp;
  std::vector<std::size_t> _next_wp;

  std::unordered_map<std::size_t,
    std::unordered_set<std::size_t>> _neighbor_map;

  double _last_update_time = 0.0;
  double _update_threshold = 0.5; // Update every 0.5s
  double _waypoint_threshold = 2.0;

  bool _merge_lane = false;
  double _lane_threshold = 0.2; // Meters

  std::string _current_task_id;

  std::mutex _graph_update_mutex;

  void map_cb(const BuildingMap::SharedPtr msg);
  void initialize_graph();
  double compute_ds(
    const Eigen::Isometry3d& pose,
    const std::size_t& wp);
  void initialize_start(const Eigen::Isometry3d& pose);
  std::size_t get_next_waypoint(const std::size_t start_wp,
    const Eigen::Vector3d& heading);
  ReadonlyCommon::Path compute_path(
    const Eigen::Isometry3d& pose);
};

template<typename SdfPtrT>
void ReadonlyCommon::read_sdf(SdfPtrT& sdf)
{
  // Getting sdf elements
  if (sdf->HasElement("level_name"))
    _level_name = sdf->template Get<std::string>("level_name");
  RCLCPP_INFO(logger(), std::string("Setting level name to: "
    + _level_name).c_str());

  if (sdf->HasElement("graph_index"))
    _nav_graph_index = sdf->template Get<std::size_t>("graph_index");
  RCLCPP_INFO(logger(), std::string("Setting nav graph index: "
    + _nav_graph_index).c_str());

  if (sdf->HasElement("spawn_waypoint"))
    _start_wp_name = sdf->template Get<std::string>("spawn_waypoint");
  RCLCPP_INFO(logger(), std::string("Setting start wp name: "
    + _start_wp_name).c_str());

  if (sdf->HasElement("look_ahead"))
    _lookahead = sdf->template Get<std::size_t>("look_ahead");
  _lookahead = _lookahead < 1 ? 1 : _lookahead;
  _next_wp.resize(_lookahead);
  _path.resize(_lookahead);
  RCLCPP_INFO(logger(), std::string("Setting lookahead: "
    + std::to_string(_lookahead)).c_str());

  if (sdf->HasElement("update_rate"))
    _update_threshold = 1.0 / sdf->template Get<double>("update_rate");
  RCLCPP_INFO(logger(),
    std::string("Setting update threshold: "
    + std::to_string(_update_threshold)).c_str());

  if (sdf->HasElement("waypoint_threshold"))
    _waypoint_threshold = sdf->template Get<double>("waypoint_threshold");
  RCLCPP_INFO(logger(),
    std::string("Setting waypoint threshold: "
    + std::to_string(_waypoint_threshold)).c_str());

  if (sdf->HasElement("merge_lane"))
    _merge_lane = sdf->template Get<bool>("merge_lane");
  RCLCPP_INFO(logger(),
    std::string("Setting merge lane: " + std::to_string(_merge_lane)).c_str());

  if (sdf->HasElement("lane_threshold"))
    _lane_threshold = sdf->template Get<double>("lane_threshold");
  RCLCPP_INFO(logger(),
    std::string("Setting lane threshold: "
    + std::to_string(_lane_threshold)).c_str());
}

} // namespace rmf_readonly_common

#endif
