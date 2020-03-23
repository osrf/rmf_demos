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
#include "RmfPanel.hpp"
#include "StandardNames.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QGridLayout>

#include <rmf_traffic/geometry/Circle.hpp>

namespace rmf_rviz_plugin {

void RmfPanel::create_layout()
{
  // Creates the layout for QT GUI
  QGroupBox *control_panel_gb = new QGroupBox("RMF Control Panel");
  QGridLayout *control_panel_layout = new QGridLayout(this);

  // Selectors 
  QGroupBox *selector_gb = new QGroupBox("Selectors");
  QGridLayout *selector_layout = new QGridLayout(this);
  selector_gb->setLayout(selector_layout);

  selector_layout->addWidget(new QLabel("Fleet: "), 0, 0); 
  _fleet_selector = new QComboBox;
  selector_layout->addWidget(_fleet_selector, 0, 1, 1, 2);

  selector_layout->addWidget(new QLabel("Robot: "), 1, 0); 
  _robot_selector = new QComboBox;
  selector_layout->addWidget(_robot_selector, 1, 1, 1, 2);

  selector_layout->addWidget(new QLabel("Start Waypoint: "), 2, 0); 
  _start_waypoint_selector = new QComboBox;
  selector_layout->addWidget(_start_waypoint_selector, 2, 1, 1, 2);
  
  selector_layout->addWidget(new QLabel("End Waypoint: "), 3, 0); 
  _end_waypoint_selector = new QComboBox;
  selector_layout->addWidget(_end_waypoint_selector, 3, 1, 1, 2);

  selector_layout->addWidget(new QLabel("Repeat Count: "), 4, 0); 
  _repeat_count_selector = new QSpinBox;
  selector_layout->addWidget(_repeat_count_selector, 4, 1, 1, 2);

  selector_layout->addWidget(new QLabel("Time: "), 5, 0); 
  _update_time_checkbox = new QCheckBox("Keep Time Updated");
  selector_layout->addWidget(_update_time_checkbox, 5, 1, 1, 2);
  _time_selector = new QTimeEdit;
  selector_layout->addWidget(_time_selector, 5, 2);

  control_panel_layout->addWidget(selector_gb);

  // Status 
  QGroupBox *status_gb = new QGroupBox("Status");
  QGridLayout *status_layout = new QGridLayout(this);
  status_gb->setLayout(status_layout);

  status_layout->addWidget(new QLabel("Fleet Status"), 0, 2); 
  _fleet_status_view = new QListView;
  status_layout->addWidget(_fleet_status_view, 1, 0, 2, 5);

  status_layout->addWidget(new QLabel("Task Summaries"), 3, 2); 
  _fleet_summary_view = new QListView;
  status_layout->addWidget(_fleet_summary_view, 4, 0, 2, 5);

  control_panel_layout->addWidget(status_gb);

  // Schedule
  QGroupBox *schedule_gb = new QGroupBox("Schedule");
  QGridLayout *schedule_layout = new QGridLayout(this);
  schedule_gb->setLayout(schedule_layout);

  _schedule_list_view = new QListView;
  schedule_layout->addWidget(_schedule_list_view, 0, 0, 10, 4);

  _pause_schedule_checkbox = new QCheckBox("Pause Schedule");
  _pause_schedule_checkbox->setChecked(true);
  schedule_layout->addWidget(_pause_schedule_checkbox, 0, 5);

  _edit_schedule_item_button = new QPushButton("Edit");
  schedule_layout->addWidget(_edit_schedule_item_button, 5, 5);

  _delete_schedule_item_button = new QPushButton("Delete");
  schedule_layout->addWidget(_delete_schedule_item_button, 6, 5);

  control_panel_layout->addWidget(schedule_gb);

  // Actions
  QGroupBox *actions_gb = new QGroupBox("Actions");
  QGridLayout *actions_layout = new QGridLayout(this);
  actions_gb->setLayout(actions_layout);

  _pause_robot_button = new QPushButton("Pause Robot");
  actions_layout->addWidget(_pause_robot_button, 0, 0, 1, 1);

  _resume_robot_button = new QPushButton("Resume Robot");
  actions_layout->addWidget(_resume_robot_button, 0, 1, 1, 1);

  _send_delivery_button = new QPushButton("Send Delivery Request");
  actions_layout->addWidget(_send_delivery_button, 1, 0, 1, 2);

  _send_loop_button = new QPushButton("Send Loop Request");
  actions_layout->addWidget(_send_loop_button, 2, 0, 1, 2);

  control_panel_layout->addWidget(actions_gb);

  // Wrap Up
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addStretch();
  layout->addWidget(control_panel_gb);
  setLayout(layout);
}
void RmfPanel::initialize_subscribers(rclcpp::Node::SharedPtr _node)
{
  _fleet_state_sub = _node->create_subscription<FleetState>(
      rmf_rviz_plugin::FleetStateTopicName, 10,
      std::bind(&RmfPanel::_fleet_state_callback, this, std::placeholders::_1));
}
void RmfPanel::initialize_state_record()
{
  // These data structures allow lookup of states of various important agents
  _map_fleet_to_robots = std::unordered_map<std::string, std::vector<std::string>>();
  _map_fleet_to_graph_info = std::unordered_map<std::string, GraphInfo>();
  _map_robot_to_state = std::unordered_map<std::string, RobotState>();
}

rmf_utils::optional<GraphInfo> RmfPanel::load_fleet_graph_info(std::string fleet_name) const 
{
  // TODO(BH): Currently mocking up VehicleTraits, potential to give more accurate values
  rclcpp::Node::SharedPtr _param_node =
      std::make_shared<rclcpp::Node>("nav_graph_param_loader");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
      _param_node, fleet_name + "_fleet_adapter");

  while (!parameters_client->wait_for_service(std::chrono::seconds(2))) {
    std::cout << "Waiting for parameter service.." << std::endl;
  }
  auto nav_graph_path_parameters =
      parameters_client->get_parameters({"nav_graph_file"});
  std::string nav_file_path = nav_graph_path_parameters[0].as_string();
  std::cout << "Nav File Path Found: " + nav_file_path << std::endl;

  auto traits = rmf_traffic::agv::VehicleTraits{
      {1.0, 1.0},
      {1.0, 1.0},
      rmf_traffic::Trajectory::Profile::make_guided(
          rmf_traffic::geometry::make_final_convex<
              rmf_traffic::geometry::Circle>(1.0))};

  rmf_utils::optional<GraphInfo> graph_info =
      parse_graph(nav_file_path, traits, *_node);
  return graph_info;
}

RmfPanel::RmfPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  _node = std::make_shared<rclcpp::Node>("rmf_panel");
  create_layout();
  initialize_subscribers(_node);
  _thread = std::thread([&]() { rclcpp::spin(_node); });

  _has_loaded = true;
}

RmfPanel::~RmfPanel()
{
  if (_has_loaded)
  {
    _thread.join();
    rclcpp::shutdown();
  }
}


// Load config
void RmfPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

// Save config
void RmfPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

// ROS2 Callbacks
void RmfPanel::_fleet_state_callback(const FleetState::SharedPtr msg) {
    //RCLCPP_INFO(_node->get_logger(), "Received FleetState!");
    auto fleet_name = msg->name;
    if (_map_fleet_to_robots.find(fleet_name) == _map_fleet_to_robots.end() )
    {
      // Fleet is new, load parameters from parameter service
      auto graph_info = load_fleet_graph_info(fleet_name);
      if (graph_info) 
      {
        // Update Fleet Graph
        _map_fleet_to_graph_info.insert(std::pair<std::string, GraphInfo>(fleet_name, graph_info.value()));
        std::vector<std::string> robots;
        for (auto robot_state : msg->robots) {
          // Record list of robots in fleet
          robots.emplace_back(robot_state.name);

          // Record state of robot
          _map_robot_to_state.insert(std::pair<std::string, RobotState>(robot_state.name, robot_state));
        }
        _map_fleet_to_robots.insert(std::pair<std::string, std::vector<std::string>>(fleet_name, robots));
      }
    }
}



} // namespace rmf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmf_rviz_plugin::RmfPanel, rviz_common::Panel)
