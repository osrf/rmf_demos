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

#include <random>

namespace rmf_rviz_plugin {

void RmfPanel::create_layout()
{
  // Creates the layout for QT GUI
  QGridLayout *control_panel_layout = new QGridLayout(this);

  // Selectors 
  QGroupBox *selector_gb = new QGroupBox("Selectors");
  QGridLayout *selector_layout = new QGridLayout();
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
  _update_time_checkbox->setChecked(true);
  selector_layout->addWidget(_update_time_checkbox, 5, 1, 1, 2);
  _time_selector = new QTimeEdit(QTime::currentTime());
  _time_selector->setDisplayFormat(QString("hh:mm:ss ap"));
  selector_layout->addWidget(_time_selector, 5, 2);

  control_panel_layout->addWidget(selector_gb, 0, 0);

  // Status 
  QGroupBox *status_gb = new QGroupBox("Status");
  QGridLayout *status_layout = new QGridLayout();
  status_gb->setLayout(status_layout);

  _fleet_summary_view = new QListView;
  status_layout->addWidget(_fleet_summary_view, 0, 0, 3, 5);

  control_panel_layout->addWidget(status_gb, 1, 0);

  // Schedule
  QGroupBox *schedule_gb = new QGroupBox("Schedule");
  QGridLayout *schedule_layout = new QGridLayout();
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

  control_panel_layout->addWidget(schedule_gb, 2, 0);

  // Actions
  QGroupBox *actions_gb = new QGroupBox("Actions");
  QGridLayout *actions_layout = new QGridLayout();
  actions_gb->setLayout(actions_layout);

  _pause_robot_button = new QPushButton("Pause Robot");
  actions_layout->addWidget(_pause_robot_button, 0, 0, 1, 1);

  _resume_robot_button = new QPushButton("Resume Robot");
  actions_layout->addWidget(_resume_robot_button, 0, 1, 1, 1);

  _send_delivery_button = new QPushButton("Send Delivery Request");
  actions_layout->addWidget(_send_delivery_button, 1, 0, 1, 2);

  _send_loop_button = new QPushButton("Send Loop Request");
  actions_layout->addWidget(_send_loop_button, 2, 0, 1, 2);

  control_panel_layout->addWidget(actions_gb, 3, 0);
}

// Initialization Functions
void RmfPanel::initialize_publishers(rclcpp::Node::SharedPtr _node)
{
  _delivery_pub = _node->create_publisher<Delivery>(
      rmf_rviz_plugin::DeliveryTopicName, rclcpp::QoS(10));
}

void RmfPanel::initialize_subscribers(rclcpp::Node::SharedPtr _node)
{
  _fleet_state_sub = _node->create_subscription<FleetState>(
      rmf_rviz_plugin::FleetStateTopicName, 10,
      std::bind(&RmfPanel::_fleet_state_callback, this, std::placeholders::_1));

  _task_summary_sub = _node->create_subscription<TaskSummary>(
      rmf_rviz_plugin::TaskSummaryTopicName, 10,
      std::bind(&RmfPanel::_task_summary_callback, this, std::placeholders::_1));
}
void RmfPanel::initialize_state_record()
{
  // These data structures allow lookup of states of various important agents
  _map_fleet_to_robots = std::unordered_map<std::string, std::vector<std::string>>();
  _map_fleet_to_graph_info = std::unordered_map<std::string, GraphInfo>();
  _map_robot_to_state = std::unordered_map<std::string, RobotState>();
}
void RmfPanel::initialize_qt_connections()
{
  connect(this, SIGNAL(configChanged()), this, SLOT(update_fleet_selector()));
  connect(this, SIGNAL(configChanged()), this, SLOT(update_robot_selector()));
  connect(_fleet_selector,
      SIGNAL(currentTextChanged(const QString&)), 
      this, 
      SLOT(update_robot_selector()));
  connect(_fleet_selector,
      SIGNAL(currentTextChanged(const QString&)), 
      this, 
      SLOT(update_start_waypoint_selector()));
  connect(_fleet_selector,
      SIGNAL(currentTextChanged(const QString&)), 
      this, 
      SLOT(update_end_waypoint_selector()));
  connect(_update_timer, SIGNAL(timeout()), this, SLOT(update_time_selector()));
  connect(_send_delivery_button, SIGNAL(clicked()), this, SLOT(send_delivery()));
  connect(_update_timer, SIGNAL(timeout()), this, SLOT(update_task_summary_list()));
}

void RmfPanel::initialize_models()
{
  _fleet_summary_model = new QStringListModel();
  _fleet_summary_data = QStringList();
  _fleet_summary_view->setModel(_fleet_summary_model);
}

// Misc Functions
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

unsigned int random_char() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);
  return dis(gen);
}

std::string RmfPanel::generate_task_uuid(int len)
{
  std::stringstream ss;
  for (auto i = 0; i < len; i++) {
    const auto rc = random_char();
    std::stringstream hexstream;
    hexstream << std::hex << rc;
    auto hex = hexstream.str();
    ss << (hex.length() < 2 ? '0' + hex : hex);
  }
  return ss.str();
}

RmfPanel::RmfPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  _node = std::make_shared<rclcpp::Node>("rmf_panel");
  _update_timer = new QTimer(this);
  _update_timer->start(1000); // Update clock running at 1Hz

  create_layout();
  initialize_subscribers(_node);
  initialize_publishers(_node);
  initialize_qt_connections();
  initialize_models();

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

// Q_SLOTS
// Actions
void RmfPanel::send_delivery()
{
  std::string start = _start_waypoint_selector->currentText().toStdString();
  std::string end = _end_waypoint_selector->currentText().toStdString();
  Delivery delivery;
  std::lock_guard<std::mutex> lock(_mutex);

  delivery.task_id = generate_task_uuid(3);
  delivery.pickup_place_name = start;
  delivery.dropoff_place_name = end;

  _delivery_pub->publish(delivery);
  RCLCPP_INFO(_node->get_logger(), "Published delivery request");
}

// Updates
void RmfPanel::update_fleet_selector()
{
  bool new_fleet_found = (_fleet_selector->count() != (int)_map_fleet_to_robots.size());
  if (new_fleet_found)
  {
    RCLCPP_INFO(_node->get_logger(), "New Fleet found, refreshing...");
    _fleet_selector->clear();
    for (auto it : _map_fleet_to_robots) {
      _fleet_selector->addItem(QString(it.first.c_str()));
    }
  }
}

void RmfPanel::update_robot_selector()
{
  std::string fleet_name = _fleet_selector->currentText().toStdString();
  auto robots = _map_fleet_to_robots[fleet_name];
  bool new_robot_found = (_robot_selector->count() != (int)robots.size());
  if (new_robot_found) {
    _robot_selector->clear();
    for (auto robot_name : robots) {
      _robot_selector->addItem(QString(robot_name.c_str()));
    }
  }
}

void RmfPanel::update_start_waypoint_selector()
{
  std::string fleet_name = _fleet_selector->currentText().toStdString();
  auto graph_info = _map_fleet_to_graph_info[fleet_name];
  _start_waypoint_selector->clear();
  for (auto waypoint : graph_info.waypoint_names) {
    _start_waypoint_selector->addItem(QString(waypoint.second.c_str()));
  }
}

void RmfPanel::update_end_waypoint_selector()
{
  std::string fleet_name = _fleet_selector->currentText().toStdString();
  auto graph_info = _map_fleet_to_graph_info[fleet_name];
  _end_waypoint_selector->clear();
  for (auto waypoint : graph_info.waypoint_names) {
    _end_waypoint_selector->addItem(QString(waypoint.second.c_str()));
  }
}

void RmfPanel::update_time_selector()
{
  if (_update_time_checkbox->isChecked())
  {
    _time_selector->setTime(QTime::currentTime());
  }
}

void RmfPanel::update_task_summary_list()
{
  _fleet_summary_model->setStringList(_fleet_summary_data);
  _fleet_summary_view->scrollToBottom();
}

// ROS2 Callbacks
void RmfPanel::_fleet_state_callback(const FleetState::SharedPtr msg) {
    //RCLCPP_INFO(_node->get_logger(), "Received FleetState!");
    bool should_update = false;
    auto fleet_name = msg->name;
    if (_map_fleet_to_robots.find(fleet_name) == _map_fleet_to_robots.end())
    {
      // Fleet is new, load parameters from parameter service
      auto graph_info = load_fleet_graph_info(fleet_name);
      if (graph_info) 
      {
        // Update Fleet Graph
        _map_fleet_to_graph_info.insert(std::pair<std::string, GraphInfo>(fleet_name, graph_info.value()));
        should_update = true;
      }
    }

    // Update robot states locally
    for (auto robot_state : msg->robots) {
      _map_robot_to_state[robot_state.name] = robot_state;
      // TODO(BH): Figure out why make_shared doesn't work?
      //auto robots = std::make_shared<std::vector<std::string>>(_map_fleet_to_robots[fleet_name]);
      auto robots = &_map_fleet_to_robots[fleet_name];
      if (std::find(robots->begin(), robots->end(), robot_state.name) == robots->end())
      {
        robots->emplace_back(robot_state.name);
        should_update = true;
      }
    }

    if (should_update)
    {
      Q_EMIT configChanged();
    }
}


void RmfPanel::_task_summary_callback(const TaskSummary::SharedPtr msg)
{
  _fleet_summary_data.append(QTime::currentTime().toString() 
      + QString("\t") 
      + QString(msg->status.c_str()));
}

} // namespace rmf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmf_rviz_plugin::RmfPanel, rviz_common::Panel)
