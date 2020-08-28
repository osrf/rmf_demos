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

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include <memory>
#include <unordered_map>
#include <mutex>

#include <gazebo/common/Plugin.hh>

#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <building_map_msgs/msg/building_map.hpp>
#include <building_map_msgs/msg/level.hpp>
#include <building_map_msgs/msg/graph.hpp>

#include "utils.hpp"

using namespace rmf_gazebo_plugins;

class ReadonlyPlugin : public gazebo::ModelPlugin
{
public:
  using BuildingMap = building_map_msgs::msg::BuildingMap;
  using Level = building_map_msgs::msg::Level;
  using Graph = building_map_msgs::msg::Graph;
  using Location = rmf_fleet_msgs::msg::Location;
  using Path = std::vector<Location>;

  ReadonlyPlugin();
  ~ReadonlyPlugin();

  void map_cb(const BuildingMap::SharedPtr msg);
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void OnUpdate();

private:

  rclcpp::Logger logger();

  void set_traits();
  void initialize_graph();
  void initialize_start(const ignition::math::Pose3d& pose);
  double compute_ds(const ignition::math::Pose3d& pose, const std::size_t& wp);
  std::size_t get_next_waypoint(const std::size_t start_wp,
    const ignition::math::Vector3d& heading);
  Path compute_path(const ignition::math::Pose3d& pose);

  gazebo::event::ConnectionPtr _update_connection;
  gazebo_ros::Node::SharedPtr _ros_node;
  gazebo::physics::ModelPtr _model;

  rclcpp::Publisher<rmf_fleet_msgs::msg::RobotState>::SharedPtr _robot_state_pub;
  rclcpp::Subscription<BuildingMap>::SharedPtr _building_map_sub;

  rmf_fleet_msgs::msg::RobotState _robot_state_msg;
  rmf_fleet_msgs::msg::RobotMode _current_mode;

  Path _path;

  bool _found_level = false;
  bool _found_graph = false;
  bool _initialized_graph = false;
  bool _initialized_start = false;

  // Store cache of BuildingMap
  // BuildingMap _map;
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
  double _lane_threshold = 0.2; // meters

  int _update_count = 0;
  std::string _name;
  std::string _current_task_id;

  std::mutex _mutex;
};

ReadonlyPlugin::ReadonlyPlugin()
{
  // We do initialization only during ::Load
}

ReadonlyPlugin::~ReadonlyPlugin()
{
}

rclcpp::Logger ReadonlyPlugin::logger()
{
  return rclcpp::get_logger("read_only_" + _model->GetName());
}

void ReadonlyPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
  _model = model;
  _ros_node = gazebo_ros::Node::Get(sdf);

  // Getting sdf elements
  if (sdf->HasElement("level_name"))
    _level_name = sdf->Get<std::string>("level_name");
  RCLCPP_INFO(logger(), "Setting level name to: " + _level_name);

  if (sdf->HasElement("graph_index"))
    _nav_graph_index = sdf->Get<std::size_t>("graph_index");
  RCLCPP_INFO(logger(), "Setting nav graph index: " + _nav_graph_index);

  if (sdf->HasElement("spawn_waypoint"))
    _start_wp_name = sdf->Get<std::string>("spawn_waypoint");
  RCLCPP_INFO(logger(), "Setting start wp name: " + _start_wp_name);

  if (sdf->HasElement("look_ahead"))
    _lookahead = sdf->Get<std::size_t>("look_ahead");
  _lookahead = _lookahead < 1 ? 1 : _lookahead;
  _next_wp.resize(_lookahead);
  _path.resize(_lookahead);
  RCLCPP_INFO(logger(), "Setting lookahead: " + std::to_string(_lookahead));

  if (sdf->HasElement("update_rate"))
    _update_threshold = 1.0 / sdf->Get<double>("update_rate");
  RCLCPP_INFO(logger(),
    "Setting update threshold: " + std::to_string(_update_threshold));

  if (sdf->HasElement("waypoint_threshold"))
    _waypoint_threshold = sdf->Get<double>("waypoint_threshold");
  RCLCPP_INFO(logger(),
    "Setting waypoint threshold: " + std::to_string(_waypoint_threshold));

  if (sdf->HasElement("merge_lane"))
    _merge_lane = sdf->Get<bool>("merge_lane");
  RCLCPP_INFO(logger(), "Setting merge lane: " + std::to_string(_merge_lane));

  if (sdf->HasElement("lane_threshold"))
    _lane_threshold = sdf->Get<double>("lane_threshold");
  RCLCPP_INFO(logger(),
    "Setting lane threshold: " + std::to_string(_lane_threshold));

  RCLCPP_INFO(logger(), "hello i am " + model->GetName());

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ReadonlyPlugin::OnUpdate, this));

  _robot_state_pub =
    _ros_node->create_publisher<rmf_fleet_msgs::msg::RobotState>(
    "/robot_state", 10);

  // Subscription to /map
  auto qos_profile = rclcpp::QoS(10);
  qos_profile.transient_local();
  _building_map_sub = _ros_node->create_subscription<BuildingMap>(
    "/map",
    qos_profile,
    std::bind(&ReadonlyPlugin::map_cb, this, std::placeholders::_1));

  _current_task_id = "demo";
  if (model->GetName().c_str())
    _name = _model->GetName().c_str();
  RCLCPP_INFO(logger(), "Setting name to: " + _name);

}

void ReadonlyPlugin::map_cb(const BuildingMap::SharedPtr msg)
{
  if (msg->levels.empty())
  {
    RCLCPP_ERROR(logger(), "Received empty building map");
    return;
  }

  RCLCPP_INFO(logger(), "Received building map with %d levels",
    msg->levels.size());
  _found_level = false;
  _found_graph = false;
  for (const auto& level: msg->levels)
  {
    RCLCPP_INFO(logger(), "Level name: [%s]", level.name.c_str());
    if (level.name.c_str() == _level_name)
    {
      _level = level;
      _found_level = true;
      RCLCPP_INFO(logger(), "Found level [%s] with %d nav_graphs",
        level.name.c_str(), level.nav_graphs.size());

      if (_nav_graph_index < level.nav_graphs.size())
      {
        _found_graph = true;
        RCLCPP_INFO(logger(), "Graph index [%d] containts [%d] waypoints",
          _nav_graph_index,
          level.nav_graphs[_nav_graph_index].vertices.size());
        initialize_graph();
      }
      else
      {
        RCLCPP_ERROR(logger(),
          "Specified nav_graph index [%d] does not exist in level [%s]",
          _nav_graph_index, _level_name.c_str());
      }
      break;
    }
  }

  if (!_found_level)
    RCLCPP_ERROR(logger(),
      "Did not find level [%s] in building map. Path will not be published.",
      _level_name.c_str());
}

void ReadonlyPlugin::initialize_graph()
{
  if (!_found_graph)
    return;

  std::lock_guard<std::mutex> lock(_mutex);

  _initialized_graph = false;

  _graph = _level.nav_graphs[_nav_graph_index];
  RCLCPP_INFO(logger(), "Nav graph contains [%d] lanes", _graph.edges.size());
  for (const auto& edge : _graph.edges)
  {
    // Inserting entry for v1_idx
    auto entry = _neighbor_map.find(edge.v1_idx);
    if (entry == _neighbor_map.end())
    {
      // We assume the vertex is a neighbor of itself to account for when the
      // robot turns around and heads back to the start index
      std::unordered_set<std::size_t> neighbors({edge.v1_idx, edge.v2_idx});
      _neighbor_map.insert(std::make_pair(edge.v1_idx, neighbors));
    }
    // Updating existing entry
    else
    {
      entry->second.insert(edge.v2_idx);
    }

    // If bidrectional, add entry for v2_idx
    if (edge.edge_type == edge.EDGE_TYPE_BIDIRECTIONAL)
    {
      entry = _neighbor_map.find(edge.v2_idx);
      if (entry == _neighbor_map.end())
      {
        std::unordered_set<std::size_t> neighbors({edge.v1_idx, edge.v2_idx});
        _neighbor_map.insert(std::make_pair(edge.v2_idx, neighbors));
      }
      // Updating existing entry
      else
      {
        entry->second.insert(edge.v1_idx);
      }
    }

  }

  _initialized_graph = true;

}

double ReadonlyPlugin::compute_ds(
  const ignition::math::Pose3d& pose,
  const std::size_t& wp)
{
  // TODO consider returning a nullptr instead
  assert(_found_graph);
  assert(wp < _graph.vertices.size());

  ignition::math::Vector3d world_position{pose.Pos().X(), pose.Pos().Y(), 0};
  ignition::math::Vector3d graph_position{
    _graph.vertices[wp].x,
    _graph.vertices[wp].y,
    0};

  return std::abs((world_position - graph_position).Length());
}

void ReadonlyPlugin::initialize_start(const ignition::math::Pose3d& pose)
{
  if (_initialized_start)
    return;

  if (!_initialized_graph)
    return;

  bool found = false;
  for (std::size_t i = 0; i < _graph.vertices.size(); i++)
  {
    if (_graph.vertices[i].name.c_str() == _start_wp_name)
    {
      found = true;
      _start_wp = i;
      RCLCPP_INFO(logger(), "Start waypoint found in nav graph");
    }
  }

  // TODO find the closest wp if the coordiantes do not match
  if (found && compute_ds(pose, _start_wp) < 1e-1)
  {
    _initialized_start = true;
    // Here we initialzie the next waypoint
    compute_path(pose);
    RCLCPP_INFO(logger(), "Start waypoint successfully initialized");
  }

  else if (found)
  {
    RCLCPP_ERROR(
      logger(),
      "Spawn coordinates [%f,%f,%f] differs from that of waypoint [%s] in nav_graph [%f, %f, %f]",
      pose.Pos().X(), pose.Pos().Y(), 0,
      _start_wp_name.c_str(),
      _graph.vertices[_start_wp].x, _graph.vertices[_start_wp].y, 0);
  }

  else
  {
    RCLCPP_ERROR(
      logger(),
      "Start waypoint [%s] not found in nav graph", _start_wp_name.c_str());
  }
}
std::size_t ReadonlyPlugin::get_next_waypoint(const std::size_t start_wp,
  const ignition::math::Vector3d& heading)
{
  // Return the waypoint closest to the robot in the direction of its heading
  const auto& neighbors = _neighbor_map.find(start_wp)->second;

  auto wp_it = neighbors.begin();
  double max_dist = std::numeric_limits<double>::min();

  for (auto it = neighbors.begin(); it != neighbors.end(); it++)
  {
    const auto& waypoint = _graph.vertices[*it];
    ignition::math::Vector3d disp_vector{
      waypoint.x - _graph.vertices[start_wp].x,
      waypoint.y - _graph.vertices[start_wp].y,
      0};
    const double dist = heading.Dot(disp_vector.Normalize());
    // Consider the waypoints with largest projected distance
    if (dist > max_dist)
    {
      max_dist = dist;
      wp_it = it;
    }
  }

  return *wp_it;
}

ReadonlyPlugin::Path ReadonlyPlugin::compute_path(
  const ignition::math::Pose3d& pose)
{
  Path path;
  path.resize(_lookahead);

  auto start_wp = _start_wp;
  const double current_yaw = pose.Rot().Euler().Z();
  ignition::math::Vector3d heading{
    std::cos(current_yaw), std::sin(current_yaw), 0.0};

  auto make_location =
    [=](double x, double y) -> Location
    {
      Location location;
      location.x = x;
      location.y = y;
      location.level_name = _level_name;

      return location;
    };

  for (std::size_t i = 0; i < _lookahead; i++)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    auto wp = get_next_waypoint(start_wp, heading);
    _next_wp[i] = wp;
    // Add to path here
    path[i] = make_location(_graph.vertices[wp].x, _graph.vertices[wp].y);
    // Update heading for next iteration
    auto next_heading = ignition::math::Vector3d{
      _graph.vertices[wp].x - _graph.vertices[start_wp].x,
      _graph.vertices[wp].y - _graph.vertices[start_wp].y,
      0};
    heading = next_heading.Normalize();
    start_wp = wp;
  }

  if (_merge_lane)
  {
    auto target = _next_wp[0];
    // Vector from target to start_wp
    auto lane_vector = ignition::math::Vector3d{
      _graph.vertices[target].x - _graph.vertices[_start_wp].x,
      _graph.vertices[target].y - _graph.vertices[_start_wp].y,
      0};

    // Vector from target to robot
    auto disp_vector = ignition::math::Vector3d{
      _graph.vertices[target].x - pose.Pos().X(),
      _graph.vertices[target].y - pose.Pos().Y(),
      0};

    // Angle between lane_vector and disp_vector
    double theta = std::atan2(lane_vector.Y(), lane_vector.X()) -
      std::atan2(disp_vector.Y(), disp_vector.X());

    // Compute the perpendicualr distance of robot from its lane
    double lane_error = std::pow(disp_vector.Length(), 2) -
      std::pow(disp_vector.Dot(lane_vector.Normalize()), 2);

    // RCLCPP_ERROR(logger(), "Disp: [%f] lane: [%f], Lane error: [%f]",
    //   disp_vector.Length(), disp_vector.Dot(lane_vector.Normalize()),lane_error);

    if (lane_error > _lane_threshold)
    {
      // TODO use tranformation matrices
      // Rotate position of robot about target by theta
      auto robot_x = pose.Pos().X() - _graph.vertices[target].x;
      auto robot_y = pose.Pos().Y() - _graph.vertices[target].y;
      robot_x = robot_x * std::cos(theta) - robot_y * std::sin(theta);
      robot_y = robot_x * std::sin(theta) + robot_y * std::cos(theta);
      robot_x += _graph.vertices[target].x;
      robot_y += _graph.vertices[target].y;

      path.insert(path.begin(), make_location(robot_x, robot_y));
    }
  }

  return path;

}

void ReadonlyPlugin::OnUpdate()
{
  _update_count++;
  const auto& world = _model->GetWorld();
  auto pose = _model->WorldPose();
  const double time = world->SimTime().Double();

  if (time - _last_update_time > _update_threshold) // todo: be smarter, use elapsed sim time
  {
    initialize_start(pose);

    _last_update_time = time;
    const int32_t t_sec = static_cast<int32_t>(time);
    const uint32_t t_nsec =
      static_cast<uint32_t>((time-static_cast<double>(t_sec)) *1e9);
    const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};

    _robot_state_msg.name = _model->GetName();
    _robot_state_msg.model = "";
    _robot_state_msg.task_id = _current_task_id;
    _robot_state_msg.mode = _current_mode;
    _robot_state_msg.battery_percent = 98.0;

    _robot_state_msg.location.x = pose.Pos().X();
    _robot_state_msg.location.y = pose.Pos().Y();
    _robot_state_msg.location.yaw = pose.Rot().Yaw();
    _robot_state_msg.location.t = now;
    _robot_state_msg.location.level_name = _level_name;

    if (_initialized_start)
    {
      if (compute_ds(pose, _next_wp[0]) <= _waypoint_threshold)
      {
        _start_wp = _next_wp[0];
        RCLCPP_INFO(logger(), "Reached waypoint [%d,%s]",
          _next_wp[0], _graph.vertices[_next_wp[0]].name.c_str());
      }
      _robot_state_msg.path = compute_path(pose);
    }

    _robot_state_pub->publish(_robot_state_msg);
  }

}

GZ_REGISTER_MODEL_PLUGIN(ReadonlyPlugin)
