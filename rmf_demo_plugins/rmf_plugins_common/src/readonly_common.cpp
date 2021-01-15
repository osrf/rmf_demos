#include <rmf_plugins_common/readonly_common.hpp>
#include <rmf_plugins_common/utils.hpp>

using namespace rmf_readonly_common;

using BuildingMap = building_map_msgs::msg::BuildingMap;
using Level = building_map_msgs::msg::Level;
using Graph = building_map_msgs::msg::Graph;
using Location = rmf_fleet_msgs::msg::Location;
using Path = std::vector<Location>;

static double compute_yaw(const Eigen::Isometry3d& pose)
{
  auto quat = Eigen::Quaterniond(pose.linear());
  // Taken from ignition math quaternion Euler()
  double yaw = std::atan2(2 * (quat.x()*quat.y() + quat.w()*quat.z()),
      (quat.w() * quat.w()) + (quat.x() * quat.x()) - (quat.y() * quat.y()) -
      (quat.z() * quat.z()));
  return yaw;
}

void ReadonlyCommon::set_name(const std::string& name)
{
  _name = name;
}

std::string ReadonlyCommon::get_name() const
{
  return _name;
}

rclcpp::Logger ReadonlyCommon::logger()
{
  return rclcpp::get_logger("read_only_" + _name);
}

void ReadonlyCommon::init(rclcpp::Node::SharedPtr node)
{
  _current_mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
  _current_task_id = "demo";

  ros_node = std::move(node);

  _robot_state_pub =
    ros_node->create_publisher<rmf_fleet_msgs::msg::RobotState>(
    "/robot_state", 10);

  auto qos_profile = rclcpp::QoS(10);
  qos_profile.transient_local();
  _building_map_sub = ros_node->create_subscription<BuildingMap>(
    "/map",
    qos_profile,
    std::bind(&ReadonlyCommon::map_cb, this, std::placeholders::_1));

  RCLCPP_INFO(logger(), std::string("hello i am " + _name).c_str());
}

void ReadonlyCommon::on_update(Eigen::Isometry3d& pose, double sim_time)
{
  _sim_time = sim_time;
  _pose = pose;

  if (_sim_time - _last_update_time > _update_threshold)
  {
    initialize_start(_pose);

    _last_update_time = _sim_time;
    const rclcpp::Time now(
      std::move(rmf_plugins_utils::simulation_now(_sim_time)));

    constexpr double battery_percent = 98.0;
    _robot_state_msg.name = _name;
    _robot_state_msg.model = "";
    _robot_state_msg.task_id = _current_task_id;
    _robot_state_msg.mode = _current_mode;
    _robot_state_msg.battery_percent = battery_percent;

    _robot_state_msg.location.x = _pose.translation()[0];
    _robot_state_msg.location.y = _pose.translation()[1];
    _robot_state_msg.location.yaw = compute_yaw(_pose);
    _robot_state_msg.location.t = now;
    _robot_state_msg.location.level_name = _level_name;

    if (_initialized_start)
    {
      if (compute_ds(_pose, _next_wp[0]) <= _waypoint_threshold)
      {
        _start_wp = _next_wp[0];
        RCLCPP_INFO(logger(), "Reached waypoint [%d,%s]",
          _next_wp[0], _graph.vertices[_next_wp[0]].name.c_str());
      }
      _robot_state_msg.path = compute_path(_pose);
    }

    _robot_state_pub->publish(_robot_state_msg);
  }
}

void ReadonlyCommon::map_cb(const BuildingMap::SharedPtr msg)
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

void ReadonlyCommon::initialize_graph()
{
  if (!_found_graph)
    return;

  std::lock_guard<std::mutex> lock(_graph_update_mutex);

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

// Returns distance between waypoint `wp` and current `pose`
double ReadonlyCommon::compute_ds(
  const Eigen::Isometry3d& pose,
  const std::size_t& wp)
{
  // TODO consider returning a nullptr instead
  assert(_found_graph);
  assert(wp < _graph.vertices.size());

  Eigen::Vector3d world_position(pose.translation()[0],
    pose.translation()[1],
    0);
  Eigen::Vector3d graph_position(
    _graph.vertices[wp].x,
    _graph.vertices[wp].y,
    0);

  return std::abs((world_position - graph_position).norm());
}

void ReadonlyCommon::initialize_start(const Eigen::Isometry3d& pose)
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

  // TODO find the closest wp if the coordinates do not match
  if (found && compute_ds(pose, _start_wp) < 1e-1)
  {
    _initialized_start = true;
    // Here we initialize the next waypoint
    compute_path(pose);
    RCLCPP_INFO(logger(), "Start waypoint successfully initialized");
  }

  else if (found)
  {
    RCLCPP_ERROR(
      logger(),
      "Spawn coordinates [%f,%f,%f] differs from that of waypoint [%s] in nav_graph [%f, %f, %f]",
      pose.translation()[0], pose.translation()[1], 0,
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

std::size_t ReadonlyCommon::get_next_waypoint(const std::size_t start_wp,
  const Eigen::Vector3d& heading)
{
  // Return the waypoint closest to the robot in the direction of its heading
  const auto& neighbors = _neighbor_map.find(start_wp)->second;

  auto wp_it = neighbors.begin();
  double max_dist = std::numeric_limits<double>::min();

  for (auto it = neighbors.begin(); it != neighbors.end(); it++)
  {
    const auto& waypoint = _graph.vertices[*it];
    Eigen::Vector3d disp_vector(
      waypoint.x - _graph.vertices[start_wp].x,
      waypoint.y - _graph.vertices[start_wp].y,
      0);

    disp_vector.normalize();
    const double dist = heading.dot(disp_vector);
    // Consider the waypoints with largest projected distance
    if (dist > max_dist)
    {
      max_dist = dist;
      wp_it = it;
    }
  }

  return *wp_it;
}

ReadonlyCommon::Path ReadonlyCommon::compute_path(
  const Eigen::Isometry3d& pose)
{
  ReadonlyCommon::Path path;
  path.resize(_lookahead);

  auto start_wp = _start_wp;
  const double current_yaw = compute_yaw(pose);
  Eigen::Vector3d heading(
    std::cos(current_yaw), std::sin(current_yaw), 0.0);

  auto make_location =
    [=](double x, double y) -> ReadonlyCommon::Location
    {
      ReadonlyCommon::Location location;
      location.x = x;
      location.y = y;
      location.level_name = _level_name;

      return location;
    };

  for (std::size_t i = 0; i < _lookahead; i++)
  {
    std::lock_guard<std::mutex> lock(_graph_update_mutex);
    auto wp = get_next_waypoint(start_wp, heading);
    _next_wp[i] = wp;
    // Add to path here
    path[i] = make_location(_graph.vertices[wp].x, _graph.vertices[wp].y);
    // Update heading for next iteration
    auto next_heading = Eigen::Vector3d(
      _graph.vertices[wp].x - _graph.vertices[start_wp].x,
      _graph.vertices[wp].y - _graph.vertices[start_wp].y,
      0);
    heading = next_heading;
    heading.normalize();
    start_wp = wp;
  }

  if (_merge_lane)
  {
    auto target = _next_wp[0];
    // Vector from target to start_wp
    auto lane_vector = Eigen::Vector3d(
      _graph.vertices[target].x - _graph.vertices[_start_wp].x,
      _graph.vertices[target].y - _graph.vertices[_start_wp].y,
      0);

    // Vector from target to robot
    auto disp_vector = Eigen::Vector3d(
      _graph.vertices[target].x - pose.translation()[0],
      _graph.vertices[target].y - pose.translation()[1],
      0);

    // Angle between lane_vector and disp_vector
    double theta = std::atan2(lane_vector(1), lane_vector(0)) -
      std::atan2(disp_vector(1), disp_vector(0));

    // Compute the perpendicular distance of robot from its lane
    lane_vector.normalize();
    double lane_error = std::pow(disp_vector.norm(), 2) -
      std::pow(disp_vector.dot(lane_vector), 2);

    // RCLCPP_ERROR(logger(), "Disp: [%f] lane: [%f], Lane error: [%f]",
    //   disp_vector.Length(), disp_vector.Dot(lane_vector.Normalize()),lane_error);

    if (lane_error > _lane_threshold)
    {
      // TODO use tranformation matrices
      // Rotate position of robot about target by theta
      auto robot_x = pose.translation()[0] - _graph.vertices[target].x;
      auto robot_y = pose.translation()[1] - _graph.vertices[target].y;
      robot_x = robot_x * std::cos(theta) - robot_y * std::sin(theta);
      robot_y = robot_x * std::sin(theta) + robot_y * std::cos(theta);
      robot_x += _graph.vertices[target].x;
      robot_y += _graph.vertices[target].y;

      path.insert(path.begin(), make_location(robot_x, robot_y));
    }
  }

  return path;
}
