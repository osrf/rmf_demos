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

#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <unordered_map>

#include <gazebo/common/Plugin.hh>

#include <building_map_msgs/msg/building_map.hpp>
#include <building_map_msgs/msg/graph.hpp>
#include <building_map_msgs/msg/level.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>

#include "utils.hpp"

using namespace rmf_gazebo_plugins;

class ReadonlyPlugin : public gazebo::ModelPlugin
{
public:
  using BuildingMap = building_map_msgs::msg::BuildingMap;
  using Level       = building_map_msgs::msg::Level;
  using Graph       = building_map_msgs::msg::Graph;
  using Location    = rmf_fleet_msgs::msg::Location;
  using Path        = std::vector<Location>;

  ReadonlyPlugin();
  ~ReadonlyPlugin();

  void map_cb(const BuildingMap::SharedPtr msg);
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void OnUpdate();

private:
  rclcpp::Logger logger();

  void set_traits();
  void initialize_graph();
  void initialize_start(const ignition::math::Pose3d &pose);
  double compute_ds(const ignition::math::Pose3d &pose, const std::size_t &wp);
  std::size_t get_next_waypoint(const std::size_t start_wp, const ignition::math::Vector3d &heading);
  Path compute_path(const ignition::math::Pose3d &pose);

  gazebo::event::ConnectionPtr _update_connection;
  gazebo_ros::Node::SharedPtr _ros_node;
  gazebo::physics::ModelPtr _model;

  rclcpp::Publisher<rmf_fleet_msgs::msg::RobotState>::SharedPtr _robot_state_pub;
  rclcpp::Subscription<BuildingMap>::SharedPtr _building_map_sub;

  rmf_fleet_msgs::msg::RobotState _robot_state_msg;
  rmf_fleet_msgs::msg::RobotMode _current_mode;

  Path _path;
  // These are currently unused but may be needed if the Planner api is used
  double _v_n     = 0.7;
  double _a_n     = 0.5;
  double _w_n     = 0.6;
  double _alpha_n = 1.5;

  bool _found_level       = false;
  bool _found_graph       = false;
  bool _initialized_graph = false;
  bool _initialized_start = false;

  // Store cache of BuildingMap
  // BuildingMap _map;
  Level _level;
  Graph _graph;

  std::string _level_name      = "L1";
  std::size_t _nav_graph_index = 1;
  std::string _start_wp_name   = "caddy";
  // The number of waypoints to add to the predicted path. Minimum is 1.
  std::size_t _lookahead = 1;
  std::size_t _start_wp;
  std::vector<std::size_t> _next_wp;

  std::unordered_map<std::size_t, std::unordered_set<std::size_t>> _neighbor_map;

  double _last_update_time = 0.0;
  int _update_count        = 0;
  std::string _name;
  std::string _current_task_id;

  std::mutex _mutex;
};

ReadonlyPlugin::ReadonlyPlugin()
{
  // We do initialization only during ::Load
}

ReadonlyPlugin::~ReadonlyPlugin() {}

rclcpp::Logger ReadonlyPlugin::logger() { return rclcpp::get_logger("read_only_" + _model->GetName()); }

void ReadonlyPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
  _model             = model;
  _ros_node          = gazebo_ros::Node::Get(sdf);

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

  if (sdf->HasElement("nominal_drive_speed"))
    _v_n = sdf->Get<double>("nominal_drive_speed");
  RCLCPP_INFO(logger(), "Setting nominal drive speed to: " + std::to_string(_v_n));

  if (sdf->HasElement("nominal_drive_acceleration"))
    _a_n = sdf->Get<double>("nominal_drive_acceleration");
  RCLCPP_INFO(logger(), "Setting nominal drive acceleration to: " + std::to_string(_a_n));

  if (sdf->HasElement("nominal_turn_speed"))
    _w_n = sdf->Get<double>("nominal_turn_speed");
  RCLCPP_INFO(logger(), "Setting nominal turn speed to:" + std::to_string(_w_n));

  if (sdf->HasElement("nominal_turn_acceleration"))
    _alpha_n = sdf->Get<double>("nominal_turn_acceleration");
  RCLCPP_INFO(logger(), "Setting nominal turn acceleration to:" + std::to_string(_alpha_n));

  RCLCPP_INFO(logger(), "hello i am " + model->GetName());

  _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ReadonlyPlugin::OnUpdate, this));

  _robot_state_pub = _ros_node->create_publisher<rmf_fleet_msgs::msg::RobotState>("/robot_state", 10);

  // Subscription to /map
  auto qos_profile = rclcpp::QoS(10);
  qos_profile.transient_local();
  _building_map_sub = _ros_node->create_subscription<BuildingMap>(
      "/map", qos_profile, std::bind(&ReadonlyPlugin::map_cb, this, std::placeholders::_1));

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

  RCLCPP_INFO(logger(), "Received building map with %d levels", msg->levels.size());
  _found_level = false;
  _found_graph = false;
  for (const auto &level : msg->levels)
  {
    RCLCPP_INFO(logger(), "Level name: [%s]", level.name.c_str());
    if (level.name.c_str() == _level_name)
    {
      _level       = level;
      _found_level = true;
      RCLCPP_INFO(logger(), "Found level [%s] with %d nav_graphs", level.name.c_str(), level.nav_graphs.size());

      if (_nav_graph_index < level.nav_graphs.size())
      {
        _found_graph = true;
        RCLCPP_INFO(logger(), "Graph index [%d] containts [%d] waypoints", _nav_graph_index,
                    level.nav_graphs[_nav_graph_index].vertices.size());
        initialize_graph();
      }
      else
      {
        RCLCPP_ERROR(logger(), "Specified nav_graph index [%d] does not exist in level [%s]", _nav_graph_index,
                     _level_name.c_str());
      }
      break;
    }
  }

  if (!_found_level)
    RCLCPP_ERROR(logger(), "Did not find level [%s] in building map. Path will not be published.", _level_name.c_str());
}

void ReadonlyPlugin::initialize_graph()
{
  if (!_found_graph)
    return;

  std::lock_guard<std::mutex> lock(_mutex);

  _initialized_graph = false;

  _graph = _level.nav_graphs[_nav_graph_index];
  RCLCPP_INFO(logger(), "Nav graph contains [%d] lanes", _graph.edges.size());
  for (const auto &edge : _graph.edges)
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

double ReadonlyPlugin::compute_ds(const ignition::math::Pose3d &pose, const std::size_t &wp)
{
  // TODO consider returning a nullptr instead
  assert(_found_graph);
  assert(wp < _graph.vertices.size());

  ignition::math::Vector3d world_position{pose.Pos().X(), pose.Pos().Y(), 0};
  ignition::math::Vector3d graph_position{_graph.vertices[wp].x, _graph.vertices[wp].y, 0};

  return std::abs((world_position - graph_position).Length());
}

void ReadonlyPlugin::initialize_start(const ignition::math::Pose3d &pose)
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
      found     = true;
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
    RCLCPP_ERROR(logger(),
                 "Spawn coordinates [%f,%f,%f] differs from that of waypoint "
                 "[%s] in nav_graph [%f, %f, %f]",
                 pose.Pos().X(), pose.Pos().Y(), 0, _start_wp_name.c_str(), _graph.vertices[_start_wp].x,
                 _graph.vertices[_start_wp].y, 0);
  }

  else
  {
    RCLCPP_ERROR(logger(), "Start waypoint [%s] not found in nav graph", _start_wp_name.c_str());
  }
}
std::size_t ReadonlyPlugin::get_next_waypoint(const std::size_t start_wp, const ignition::math::Vector3d &heading)
{
  // Return the waypoint closest to the robot in the direction of its heading
  const auto &neighbors = _neighbor_map.find(start_wp)->second;

  auto wp_it      = neighbors.begin();
  double max_dist = std::numeric_limits<double>::min();

  for (auto it = neighbors.begin(); it != neighbors.end(); it++)
  {
    const auto &waypoint = _graph.vertices[*it];
    ignition::math::Vector3d disp_vector{waypoint.x - _graph.vertices[start_wp].x,
                                         waypoint.y - _graph.vertices[start_wp].y, 0};
    const double dist = heading.Dot(disp_vector.Normalize());
    // Consider the waypoints with largest projected distance
    if (dist > max_dist)
    {
      max_dist = dist;
      wp_it    = it;
    }
  }

  return *wp_it;
}

ReadonlyPlugin::Path ReadonlyPlugin::compute_path(const ignition::math::Pose3d &pose)
{
  Path path;
  path.resize(_lookahead);

  auto start_wp            = _start_wp;
  const double current_yaw = pose.Rot().Euler().Z();
  ignition::math::Vector3d heading{std::cos(current_yaw), std::sin(current_yaw), 0.0};

  for (std::size_t i = 0; i < _lookahead; i++)
  {
    std::lock_guard<std::mutex> lock(_mutex);
    auto wp     = get_next_waypoint(start_wp, heading);
    _next_wp[i] = wp;
    // Add to path here
    Location location;
    location.x          = _graph.vertices[wp].x;
    location.y          = _graph.vertices[wp].y;
    location.level_name = _level_name;
    path[i]             = location;
    // Update heading for next iteration
    auto next_heading = ignition::math::Vector3d{_graph.vertices[wp].x - _graph.vertices[start_wp].x,
                                                 _graph.vertices[wp].y - _graph.vertices[start_wp].y, 0};
    heading           = next_heading.Normalize();
    start_wp          = wp;
  }

  return path;
}

void ReadonlyPlugin::OnUpdate()
{
  _update_count++;
  const auto &world = _model->GetWorld();
  auto pose         = _model->WorldPose();

  if (_update_count % 100 == 0) // todo: be smarter, use elapsed sim time
  {
    initialize_start(pose);

    const double time     = world->SimTime().Double();
    _last_update_time     = time;
    const int32_t t_sec   = static_cast<int32_t>(time);
    const uint32_t t_nsec = static_cast<uint32_t>((time - static_cast<double>(t_sec)) * 1e9);
    const rclcpp::Time now{t_sec, t_nsec, RCL_ROS_TIME};

    _robot_state_msg.name            = _model->GetName();
    _robot_state_msg.model           = "";
    _robot_state_msg.task_id         = _current_task_id;
    _robot_state_msg.mode            = _current_mode;
    _robot_state_msg.battery_percent = 98.0;

    _robot_state_msg.location.x          = pose.Pos().X();
    _robot_state_msg.location.y          = pose.Pos().Y();
    _robot_state_msg.location.yaw        = pose.Rot().Yaw();
    _robot_state_msg.location.t          = now;
    _robot_state_msg.location.level_name = _level_name;

    if (_initialized_start)
    {
      if (compute_ds(pose, _next_wp[0]) <= 2.0)
      {
        _start_wp = _next_wp[0];
        RCLCPP_INFO(logger(), "Reached waypoint [%d,%s]", _next_wp[0], _graph.vertices[_next_wp[0]].name.c_str());
      }
      _robot_state_msg.path = compute_path(pose);
    }

    _robot_state_pub->publish(_robot_state_msg);
  }
}

GZ_REGISTER_MODEL_PLUGIN(ReadonlyPlugin)
