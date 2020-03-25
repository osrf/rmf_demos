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

  // Options
  QGroupBox *options_gb = new QGroupBox("Options");
  QGridLayout *options_layout = new QGridLayout();
  options_gb->setLayout(options_layout);

  // Keeps the time selector at current time
  _update_time_checkbox = new QCheckBox("Keep Time Updated");
  _update_time_checkbox->setChecked(true);
  options_layout->addWidget(_update_time_checkbox, 0, 0);

  _pause_schedule_checkbox = new QCheckBox("Pause Schedule");
  _pause_schedule_checkbox->setChecked(true);
  options_layout->addWidget(_pause_schedule_checkbox, 0, 1);

  _workcells_only_checkbox = new QCheckBox("Workcells Only");
  options_layout->addWidget(_workcells_only_checkbox, 0, 2);

  control_panel_layout->addWidget(options_gb, 0, 0);

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
  _start_waypoint_selector->setEditable(true);
  selector_layout->addWidget(_start_waypoint_selector, 2, 1, 1, 2);
  
  selector_layout->addWidget(new QLabel("End Waypoint: "), 3, 0); 
  _end_waypoint_selector = new QComboBox;
  _end_waypoint_selector->setEditable(true);
  selector_layout->addWidget(_end_waypoint_selector, 3, 1, 1, 2);

  selector_layout->addWidget(new QLabel("Repeat Count: "), 4, 0); 
  _repeat_count_selector = new QSpinBox;
  selector_layout->addWidget(_repeat_count_selector, 4, 1, 1, 2);

  selector_layout->addWidget(new QLabel("Time: "), 5, 0); 
  _time_selector = new QTimeEdit(QTime::currentTime());
  _time_selector->setDisplayFormat(QString("hh:mm:ss ap"));
  selector_layout->addWidget(_time_selector, 5, 1, 1, 2);

  control_panel_layout->addWidget(selector_gb, 1, 0);

  // Status 
  QGroupBox *status_gb = new QGroupBox("Status");
  QGridLayout *status_layout = new QGridLayout();
  status_gb->setLayout(status_layout);

  _fleet_summary_view = new QListView;
  status_layout->addWidget(_fleet_summary_view, 0, 0, 3, 5);

  control_panel_layout->addWidget(status_gb, 2, 0);

  // Schedule
  QGroupBox *schedule_gb = new QGroupBox("Schedule");
  QGridLayout *schedule_layout = new QGridLayout();
  schedule_gb->setLayout(schedule_layout);

  _schedule_list_view = new QListView;
  schedule_layout->addWidget(_schedule_list_view, 0, 0, 9, 2);

  _edit_schedule_item_button = new QPushButton("Edit");
  schedule_layout->addWidget(_edit_schedule_item_button, 10, 0, 1, 1);

  _delete_schedule_item_button = new QPushButton("Delete");
  schedule_layout->addWidget(_delete_schedule_item_button, 10, 1, 1, 1);

  control_panel_layout->addWidget(schedule_gb, 3, 0);

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

  control_panel_layout->addWidget(actions_gb, 4, 0);
}

// Initialization Functions
void RmfPanel::initialize_publishers(rclcpp::Node::SharedPtr _node)
{
  _delivery_pub = _node->create_publisher<Delivery>(
      rmf_rviz_plugin::DeliveryTopicName, rclcpp::QoS(10));

  _loop_pub = _node->create_publisher<Loop>(
      rmf_rviz_plugin::LoopRequestTopicName, rclcpp::QoS(10));

  // TODO: Simulation robots do not seem to respond?
  _mode_request_pub = _node->create_publisher<ModeRequest>(
      rmf_rviz_plugin::ModeRequestTopicName, rclcpp::QoS(10));
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
  connect(this, SIGNAL(configChanged()), this, SLOT(update_schedule()));

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
  connect(_update_timer, SIGNAL(timeout()), this, SLOT(update_task_summary_list()));
  connect(_update_timer, SIGNAL(timeout()), this, SLOT(pop_schedule()));

  connect(_send_delivery_button, SIGNAL(clicked()), this, SLOT(queue_delivery()));
  connect(_send_loop_button, SIGNAL(clicked()), this, SLOT(queue_loop()));
  connect(_delete_schedule_item_button, SIGNAL(clicked()), this, SLOT(delete_schedule_item()));
  connect(_pause_robot_button, SIGNAL(clicked()), this, SLOT(pause_robot()));
  connect(_resume_robot_button, SIGNAL(clicked()), this, SLOT(resume_robot()));

  connect(_workcells_only_checkbox, SIGNAL(stateChanged(int)), 
      this, SLOT(update_start_waypoint_selector()));
  connect(_workcells_only_checkbox, SIGNAL(stateChanged(int)), 
      this, SLOT(update_end_waypoint_selector()));
}

void RmfPanel::initialize_models()
{
  _fleet_summary_model = new QStringListModel();
  _fleet_summary_data = QStringList();
  _fleet_summary_view->setModel(_fleet_summary_model);

  _schedule_list_model = new QStringListModel();
  _schedule_list_data = QStringList();
  _schedule_list_view->setModel(_schedule_list_model);

  _queued_deliveries = std::vector<std::pair<QTime, Delivery>>();
  _queued_loops = std::vector<std::pair<QTime, Loop>>();
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

bool RmfPanel::waypoint_has_workcell(std::string waypoint_name, GraphInfo& graph_info)
{
  auto idx = graph_info.keys.find(waypoint_name);
  if (idx == graph_info.keys.end())
  {
    RCLCPP_ERROR(_node->get_logger(), "Provided graph does not have this waypoint.");
    return false;
  }
  if (graph_info.workcell_names.find(idx->second) == graph_info.workcell_names.end())
  {
    // Workcell does not exist at this waypoint
    return false;
  }
  else
  {
    return true;
  }
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
void RmfPanel::queue_delivery()
{
  std::string start = _start_waypoint_selector->currentText().toStdString();
  std::string end = _end_waypoint_selector->currentText().toStdString();
  Delivery delivery;

  delivery.task_id = generate_task_uuid(3);
  delivery.pickup_place_name = start;
  delivery.dropoff_place_name = end;

  auto delivery_time = _time_selector->time();

  int insertPos = 0;
  for(auto it = _queued_deliveries.begin(); it != _queued_deliveries.end(); it++)
  {
    if (delivery_time <= it->first)
    {
      break;
    }
    else
    {
      insertPos++;
    }
  }

  std::pair<QTime, Delivery> data = std::pair<QTime, Delivery>(delivery_time, delivery);
  _queued_deliveries.insert(_queued_deliveries.begin() + insertPos, data);

  RCLCPP_INFO(_node->get_logger(), "Queued delivery request");
  Q_EMIT configChanged();
}

void RmfPanel::queue_loop()
{
  std::string start = _start_waypoint_selector->currentText().toStdString();
  std::string end = _end_waypoint_selector->currentText().toStdString();
  Loop loop;

  loop.task_id = generate_task_uuid(4);
  loop.robot_type = _fleet_selector->currentText().toStdString();
  loop.num_loops = _repeat_count_selector->value();
  loop.start_name = start;
  loop.finish_name = end;

  auto loop_time = _time_selector->time();

  int insertPos = 0;
  for(auto it = _queued_loops.begin(); it != _queued_loops.end(); it++)
  {
    if (loop_time <= it->first)
    {
      break;
    }
    else
    {
      insertPos++;
    }
  }

  std::pair<QTime, Loop> data = std::pair<QTime, Loop>(loop_time, loop);
  _queued_loops.insert(_queued_loops.begin() + insertPos, data);

  RCLCPP_INFO(_node->get_logger(), "Queued loop request");
  Q_EMIT configChanged();
}


void RmfPanel::pop_delivery()
{
  auto msg = _queued_deliveries.begin()->second;
  std::lock_guard<std::mutex> lock(_mutex);
  _delivery_pub->publish(msg);
  _queued_deliveries.erase(_queued_deliveries.begin());
}

void RmfPanel::pop_loop()
{
  auto msg = _queued_loops.begin()->second;
  std::lock_guard<std::mutex> lock(_mutex);
  _loop_pub->publish(msg);
  _queued_loops.erase(_queued_loops.begin());
}

void RmfPanel::delete_schedule_item()
{
  if(_schedule_list_data.count() == 0)
  {
    return;
  }

  int idx = _schedule_list_view->currentIndex().row();

  // These index the offset into the respective vectors we are aiming to delete
  int d_idx = 0;
  int l_idx = 0;
  bool is_loop = true;
  std::cout << "Index" << idx << std::endl;
  for(int i=0; i <= idx; i++) // We need to iterate at least once
  {
    // Iterate through the selection index from the listview, pointing to the 
    // corresponding element in the right vector according to increasing time
    if (_queued_deliveries.begin() + d_idx == _queued_deliveries.end())
    {
      // deliveries is empty, increment loops
      l_idx++;
      is_loop = true;
    }
    else if (_queued_loops.begin() + l_idx == _queued_loops.end())
    {
      // loops is empty ,increment deliveries
      d_idx++;
      is_loop = false;
    }
    else if (_queued_loops[l_idx].first <= _queued_deliveries[d_idx].first)
    {
      // Loops has earlier timestamp, increment loops
      l_idx++;
      is_loop = true;
    }
    else 
    {
      // Deliveries has earlier timestamp, increment deliveries
      d_idx++;
      is_loop = false;
    }
  }

  // Once we iterate through all i, we know what to delete
  if (is_loop)
  {
    _queued_loops.erase(_queued_loops.begin() + l_idx - 1);
  } 
  else
  {
    _queued_deliveries.erase(_queued_deliveries.begin() + d_idx - 1);
  }
  Q_EMIT configChanged();
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

void RmfPanel::pause_robot()
{
  ModeRequest msg = ModeRequest();
  msg.fleet_name = _fleet_selector->currentText().toStdString();
  msg.robot_name = _robot_selector->currentText().toStdString();
  msg.task_id = generate_task_uuid(2);
  msg.mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_PAUSED;
  _mode_request_pub->publish(msg);
  RCLCPP_INFO(_node->get_logger(), "Pausing robot..");
}

void RmfPanel::resume_robot()
{
  ModeRequest msg = ModeRequest();
  msg.fleet_name = _fleet_selector->currentText().toStdString();
  msg.robot_name = _robot_selector->currentText().toStdString();
  msg.task_id = generate_task_uuid(2);
  msg.mode.mode = rmf_fleet_msgs::msg::RobotMode::MODE_MOVING;
  _mode_request_pub->publish(msg);
  RCLCPP_INFO(_node->get_logger(), "Resuming robot..");
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
    if (!_workcells_only_checkbox->isChecked() 
        || waypoint_has_workcell(waypoint.second, graph_info))
    _start_waypoint_selector->addItem(QString(waypoint.second.c_str()));
  }
}

void RmfPanel::update_end_waypoint_selector()
{
  std::string fleet_name = _fleet_selector->currentText().toStdString();
  auto graph_info = _map_fleet_to_graph_info[fleet_name];
  _end_waypoint_selector->clear();
  for (auto waypoint : graph_info.waypoint_names) {
    if (!_workcells_only_checkbox->isChecked() 
        || waypoint_has_workcell(waypoint.second, graph_info))
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

void RmfPanel::update_schedule()
{
  int queued_deliveries_count = _queued_deliveries.size();
  int queued_loops_count = _queued_loops.size();
  int schedule_count = _schedule_list_data.size();
  bool schedule_changed = (schedule_count != queued_deliveries_count + queued_loops_count);
  bool schedule_empty = (_queued_deliveries.size() + _queued_loops.size()) == 0;

  _schedule_list_data = QStringList();

  if (schedule_empty)
  {
  }
  else if (schedule_changed) 
  {
    // First clear all data

    // Iterate down _queued_deliveries and _queued_loops, appending them to the schedule
    auto deliver_it = _queued_deliveries.cbegin();
    auto loop_it = _queued_loops.cbegin();
    QTime ref_time;
    
    // Initialize reference time
    if(loop_it == _queued_loops.cend())
    {
      ref_time = deliver_it->first;
    } 
    else if (deliver_it == _queued_deliveries.cend()) {
      ref_time = loop_it->first;
    }
    else if (deliver_it->first <= loop_it->first)
    {
      ref_time = deliver_it->first;
    }
    else
    {
      ref_time = loop_it->first;
    }

    while(deliver_it != _queued_deliveries.cend() || loop_it != _queued_loops.cend())
    {
      // Iterate over deliveries and loops, appending to schedule list in chronological order
      bool add_from_del = false;
      bool add_from_loop = false;
      if (deliver_it == _queued_deliveries.cend())
      {
        // deliveries is empty, add from loops
        add_from_loop = true;
      }
      else if (loop_it == _queued_loops.cend())
      {
        add_from_del = true;
      }
      else if (deliver_it->first < loop_it->first)
      {
        // If delivery time is earlier, queue it first
        add_from_del = true;
      }
      else if (loop_it->first <= deliver_it->first)
      {
        // If loop time is earlier, queue it first
        add_from_loop = true;
      }
      
      // Finally, decide what to do this loop
      if (add_from_loop)
      {
        std::stringstream ss;
        ss << "\tLoop\t"
           << "From: " + loop_it->second.start_name + "\t"
           << "To: " + loop_it->second.finish_name + "\t"
           << "Repeat: " + std::to_string(loop_it->second.num_loops)
           << std::endl;
        _schedule_list_data.append(loop_it->first.toString() 
            + QString(ss.str().c_str()));
        loop_it++;
      } 
      else if (add_from_del)
      {
        std::stringstream ss;
        ss << "\tDelivery\t"
           << "From: " + deliver_it->second.pickup_place_name + "\t"
           << "To: " + deliver_it->second.dropoff_place_name
           << std::endl;
        _schedule_list_data.append(deliver_it->first.toString() 
            + QString(ss.str().c_str()));
        deliver_it++;
      }
    }
  }
  for (auto i : _schedule_list_data) {
      std::cout << i.toStdString() << std::endl;
  }
  _schedule_list_model->setStringList(_schedule_list_data);
}

void RmfPanel::pop_schedule()
{
  if (_pause_schedule_checkbox->isChecked())
  {
    // If schedule is paused, we will not execute any schedule items.
    return;
  }

  for(auto delivery_data : _queued_deliveries)
  {
    if (QTime::currentTime() > delivery_data.first)
    {
      pop_delivery();
      Q_EMIT configChanged();
    } 
    else 
    {
      break;
    }
  }

  for(auto loop_data : _queued_loops)
  {
    if (QTime::currentTime() > loop_data.first)
    {
      pop_loop();
      Q_EMIT configChanged();
    } 
    else 
    {
      break;
    }
  }
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
