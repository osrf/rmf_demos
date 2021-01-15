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
#include "ParseActionPlan.hpp"

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include <rmf_traffic/geometry/Circle.hpp>

#include <random>

namespace rmf_rviz_plugin {

using Delivery = rmf_task_msgs::msg::Delivery;
using Loop = rmf_task_msgs::msg::Loop;
using FleetState = rmf_fleet_msgs::msg::FleetState;
using RobotState = rmf_fleet_msgs::msg::RobotState;
using DoorState = rmf_door_msgs::msg::DoorState;
using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
using Location = rmf_fleet_msgs::msg::Location;
using DoorMode = rmf_door_msgs::msg::DoorMode;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using PointStamped = geometry_msgs::msg::PointStamped;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using GetParameters = rcl_interfaces::srv::GetParameters;
using PathRequest = rmf_fleet_msgs::msg::PathRequest;
using ModeRequest = rmf_fleet_msgs::msg::ModeRequest;
using DoorRequest = rmf_door_msgs::msg::DoorRequest;
using TaskSummary = rmf_task_msgs::msg::TaskSummary;
using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
using Graph = rmf_traffic::agv::Graph;
using Bool = std_msgs::msg::Bool;

void RmfPanel::create_layout()
{
  // Creates the layout for QT GUI
  QGridLayout* control_panel_layout = new QGridLayout(this);

  // Selectors
  QGroupBox* selector_gb = new QGroupBox("Selectors");
  QGridLayout* selector_layout = new QGridLayout();
  selector_gb->setLayout(selector_layout);

  selector_layout->addWidget(new QLabel("Fleet: "), 0, 0);
  _fleet_selector = new QComboBox;
  selector_layout->addWidget(_fleet_selector, 0, 1, 1, 2);

  selector_layout->addWidget(new QLabel("Start Waypoint: "), 1, 0);
  _start_waypoint_selector = new QComboBox;
  _start_waypoint_selector->setEditable(true);
  selector_layout->addWidget(_start_waypoint_selector, 1, 1, 1, 2);

  _workcells_only_checkbox = new QCheckBox("Workcells Only");
  selector_layout->addWidget(_workcells_only_checkbox, 1, 3);

  selector_layout->addWidget(new QLabel("End Waypoint: "), 2, 0);
  _end_waypoint_selector = new QComboBox;
  _end_waypoint_selector->setEditable(true);
  selector_layout->addWidget(_end_waypoint_selector, 2, 1, 1, 2);

  selector_layout->addWidget(new QLabel("Time: "), 3, 0);
  _time_selector = new QTimeEdit(QTime::currentTime());
  _time_selector->setDisplayFormat(QString("hh:mm:ss ap"));
  selector_layout->addWidget(_time_selector, 3, 1, 1, 2);

  control_panel_layout->addWidget(selector_gb, 1, 0);

  _update_time_checkbox = new QCheckBox("Keep Time Updated");
  _update_time_checkbox->setChecked(true);
  selector_layout->addWidget(_update_time_checkbox, 3, 3);

  // Loop Action
  QGroupBox* loop_gb = new QGroupBox("Loop Request");
  QGridLayout* loop_layout = new QGridLayout();
  loop_gb->setLayout(loop_layout);
  loop_layout->addWidget(new QLabel("# loops: "), 0, 0);

  _repeat_count_selector = new QSpinBox;
  _repeat_count_selector->setValue(1);
  loop_layout->addWidget(_repeat_count_selector, 0, 1, 1, 2);

  _send_loop_button = new QPushButton("Send Loop Request");
  loop_layout->addWidget(_send_loop_button, 1, 0, 1, -1);

  control_panel_layout->addWidget(loop_gb, 2, 0);

  // Delivery Action
  QGroupBox* delivery_gb = new QGroupBox("Delivery Request");
  QGridLayout* delivery_layout = new QGridLayout();
  delivery_gb->setLayout(delivery_layout);

  delivery_layout->addWidget(new QLabel("Pickup dispenser: "), 0, 0);
  _pickup_dispenser_editor = new QLineEdit;
  delivery_layout->addWidget(_pickup_dispenser_editor, 0, 1, 1, 2);

  delivery_layout->addWidget(new QLabel("Dropoff ingestor: "), 1, 0);
  _dropoff_ingestor_editor = new QLineEdit;
  delivery_layout->addWidget(_dropoff_ingestor_editor, 1, 1, 1, 2);

  _send_delivery_button = new QPushButton("Send Delivery Request");
  delivery_layout->addWidget(_send_delivery_button, 2, 0, 1, -1);

  control_panel_layout->addWidget(delivery_gb, 3, 0);

  // Emergency Alarm
  QGroupBox* emergency_gb = new QGroupBox("Emergency Alarm");
  QGridLayout* emergency_layout = new QGridLayout();
  emergency_gb->setLayout(emergency_layout);

  _emergency_state_checkbox = new QCheckBox("Activate Alarm");
  emergency_layout->addWidget(_emergency_state_checkbox, 0, 1, 1, 3);

  control_panel_layout->addWidget(emergency_gb, 4, 0);

  // Status
  QGroupBox* status_gb = new QGroupBox("Status");
  QGridLayout* status_layout = new QGridLayout();
  status_gb->setLayout(status_layout);

  _fleet_summary_view = new QListView;
  status_layout->addWidget(_fleet_summary_view, 0, 0, 3, 5);

  control_panel_layout->addWidget(status_gb, 5, 0);

  // Plan
  QGroupBox* plan_gb = new QGroupBox("Plan");
  QGridLayout* plan_layout = new QGridLayout();
  plan_gb->setLayout(plan_layout);

  _pause_plan_checkbox = new QCheckBox("Pause Plan");
  _pause_plan_checkbox->setChecked(false);
  plan_layout->addWidget(_pause_plan_checkbox, 0, 1);

  _plan_list_view = new QListView;
  plan_layout->addWidget(_plan_list_view, 1, 0, 9, 2);

  _load_action_plan_button = new QPushButton("Load");
  plan_layout->addWidget(_load_action_plan_button, 11, 0, 1, 1);

  _delete_plan_item_button = new QPushButton("Delete");
  plan_layout->addWidget(_delete_plan_item_button, 11, 1, 1, 1);

  control_panel_layout->addWidget(plan_gb, 6, 0);

  // Load File Dialog
  _load_file_dialog = new QFileDialog();

}

// Initialization Functions
void RmfPanel::initialize_publishers()
{
  _delivery_pub = _node->create_publisher<Delivery>(
    rmf_rviz_plugin::DeliveryTopicName, rclcpp::QoS(10));

  _loop_pub = _node->create_publisher<Loop>(
    rmf_rviz_plugin::LoopRequestTopicName, rclcpp::QoS(10));

  _emergency_state_pub = _node->create_publisher<Bool>(
    rmf_rviz_plugin::EmergencyStateTopicName, rclcpp::QoS(10));
}

void RmfPanel::initialize_subscribers()
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
  _map_fleet_to_robots =
    std::unordered_map<std::string, std::vector<std::string>>();
  _map_fleet_to_graph_info = std::unordered_map<std::string, GraphInfo>();
  _map_robot_to_state = std::unordered_map<std::string, RobotState>();
}
void RmfPanel::initialize_qt_connections()
{
  connect(this, SIGNAL(configChanged()), this, SLOT(update_fleet_selector()));
  connect(this, SIGNAL(configChanged()), this, SLOT(update_plan()));

  connect(_fleet_selector, SIGNAL(currentTextChanged(const QString&)), this,
    SLOT(update_start_waypoint_selector()));
  connect(_fleet_selector, SIGNAL(currentTextChanged(const QString&)), this,
    SLOT(update_end_waypoint_selector()));

  connect(_update_timer, SIGNAL(timeout()), this, SLOT(update_time_selector()));
  connect(_update_timer, SIGNAL(timeout()), this,
    SLOT(update_task_summary_list()));
  connect(_update_timer, SIGNAL(timeout()), this, SLOT(pop_plan()));
  connect(_update_timer, SIGNAL(timeout()), this,
    SLOT(publish_emergency_signal()));

  connect(_send_delivery_button, SIGNAL(clicked()), this,
    SLOT(queue_delivery()));
  connect(_send_loop_button, SIGNAL(clicked()), this, SLOT(queue_loop()));
  connect(_delete_plan_item_button, SIGNAL(clicked()), this,
    SLOT(delete_plan_item()));

  connect(_workcells_only_checkbox, SIGNAL(stateChanged(int)), this,
    SLOT(update_start_waypoint_selector()));
  connect(_workcells_only_checkbox, SIGNAL(stateChanged(int)), this,
    SLOT(update_end_waypoint_selector()));

  connect(_load_action_plan_button, SIGNAL(clicked()),
    this, SLOT(open_load_file_dialog()));
  connect(_load_file_dialog, SIGNAL(fileSelected(const QString&)),
    this, SLOT(load_plan_from_file(const QString&)));

}

void RmfPanel::initialize_models()
{
  _fleet_summary_model = new QStringListModel();
  _fleet_summary_data = QStringList();
  _fleet_summary_view->setModel(_fleet_summary_model);

  _plan_list_model = new QStringListModel();
  _plan_list_data = QStringList();
  _plan_list_view->setModel(_plan_list_model);

  _queued_deliveries = std::vector<std::pair<QTime, Delivery>>();
  _queued_loops = std::vector<std::pair<QTime, Loop>>();
}

// Misc Functions
rmf_utils::optional<GraphInfo>
RmfPanel::load_fleet_graph_info(std::string fleet_name) const
{
  // TODO(BH): Currently mocking up VehicleTraits, potential to give more
  // accurate values

  RCLCPP_INFO(_node->get_logger(),
    std::string("Loading " + fleet_name + "..").c_str());

  rclcpp::Node::SharedPtr _param_node =
    std::make_shared<rclcpp::Node>("nav_graph_param_loader");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
    _param_node, fleet_name + "_fleet_adapter");

  // Wait for service to be available. After a cutoff duration, we can
  // conclude that the fleet_adapter was named wrongly
  if (!parameters_client->wait_for_service(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(
      _node->get_logger(),
      std::string("Parameter service not found for "
      + fleet_name + ".").c_str());
    RCLCPP_ERROR(
      _node->get_logger(),
      std::string("Please check that the fleet adapter is named "
      + fleet_name + "_fleet_adapter").c_str());
    return rmf_utils::nullopt;
  }

  // Try to load nav graph
  try
  {
    auto nav_graph_path_parameters =
      parameters_client->get_parameters({"nav_graph_file"});
    std::string nav_file_path = nav_graph_path_parameters[0].as_string();
    std::cout << "Nav File Path Found: " + nav_file_path << std::endl;

    const auto footprint = rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0);
    auto traits = rmf_traffic::agv::VehicleTraits{
      {1.0, 1.0},
      {1.0, 1.0},
      {footprint}};

    rmf_utils::optional<GraphInfo> graph_info =
      parse_graph(nav_file_path, traits, *_node);
    return graph_info;
  }

  // If the nav graph is not available, these should help debug.
  catch (rclcpp::ParameterTypeException& e)
  {
    RCLCPP_INFO(_node->get_logger(), "Nav File not found. \n");
    RCLCPP_INFO(_node->get_logger(),
      "If this adapter is Read Only, this is fine. \n");
    RCLCPP_INFO(_node->get_logger(),
      "If this adapter is Full Control, this should not happen. \n");
    RCLCPP_INFO(_node->get_logger(),
      "Check that the launch file parameter 'nav_graph_file' is correct. \n");
    return rmf_utils::nullopt;
  }
}

unsigned int random_char()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);
  return dis(gen);
}

std::string RmfPanel::generate_task_uuid(const int len)
{
  std::stringstream ss;
  for (auto i = 0; i < len; i++)
  {
    const auto rc = random_char();
    std::stringstream hexstream;
    hexstream << std::hex << rc;
    auto hex = hexstream.str();
    ss << (hex.length() < 2 ? '0' + hex : hex);
  }
  return ss.str();
}

bool RmfPanel::waypoint_has_workcell(const std::string waypoint_name,
  const GraphInfo& graph_info)
{
  auto idx = graph_info.graph.keys().find(waypoint_name);
  if (idx == graph_info.graph.keys().end())
  {
    RCLCPP_ERROR(_node->get_logger(),
      "Provided graph does not have this waypoint.");
    return false;
  }
  if (graph_info.workcell_names.find(idx->second) ==
    graph_info.workcell_names.end()) \
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
  initialize_subscribers();
  initialize_publishers();
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
  QString pickup_dispenser;
  QString dropoff_ingestor;
  if (config.mapGetString("pickup_dispenser", &pickup_dispenser))
    _pickup_dispenser_editor->setText(pickup_dispenser);
  if (config.mapGetString("dropoff_ingestor", &dropoff_ingestor))
    _dropoff_ingestor_editor->setText(dropoff_ingestor);
}

// Save config
void RmfPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("pickup_dispenser", _pickup_dispenser_editor->text());
  config.mapSetValue("dropoff_ingestor", _dropoff_ingestor_editor->text());
}

// Q_SLOTS
// Actions
void RmfPanel::queue_delivery()
{
  std::string start = _start_waypoint_selector->currentText().toStdString();
  std::string end = _end_waypoint_selector->currentText().toStdString();
  std::string pickup_dispenser = _pickup_dispenser_editor->text().toStdString();
  std::string dropoff_ingestor =
    _dropoff_ingestor_editor->text().toStdString();

  Delivery delivery;

  // If either start or end are empty, the delivery should probably not be queued
  if (start.empty() || end.empty() || pickup_dispenser.empty()
    || dropoff_ingestor.empty())
  {
    RCLCPP_INFO(_node->get_logger(),
      "Waypoint input is empty string; Delivery not queued in plan.");
    return;
  }

  delivery.task_id = generate_task_uuid(3);
  delivery.pickup_place_name = start;
  delivery.dropoff_place_name = end;
  delivery.pickup_dispenser = pickup_dispenser;
  delivery.dropoff_ingestor = dropoff_ingestor;

  auto delivery_time = _time_selector->time();

  // _queued_deliveries may be mutated asynchronously by functions like pop_delivery
  // Thus a mutex will help prevent issues
  std::unique_lock<std::mutex> lock(_mutex);


  int insertPos = 0;
  for (auto it = _queued_deliveries.begin(); it != _queued_deliveries.end();
    it++)
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

  std::pair<QTime, Delivery> data =
    std::pair<QTime, Delivery>(delivery_time, delivery);
  _queued_deliveries.insert(_queued_deliveries.begin() + insertPos, data);

  RCLCPP_INFO(_node->get_logger(), "Queued delivery request");
  lock.unlock();

  Q_EMIT configChanged();
}

void RmfPanel::queue_loop()
{
  std::string start = _start_waypoint_selector->currentText().toStdString();
  std::string end = _end_waypoint_selector->currentText().toStdString();
  Loop loop;

  // If either start or end are empty, the delivery should probably not be queued
  if (start.empty() || end.empty())
  {
    RCLCPP_INFO(
      _node->get_logger(),
      "Waypoint input is empty string; Delivery not queued in plan.");
    return;
  }

  loop.task_id = generate_task_uuid(4);
  loop.robot_type = _fleet_selector->currentText().toStdString();
  loop.num_loops = _repeat_count_selector->value();
  loop.start_name = start;
  loop.finish_name = end;

  auto loop_time = _time_selector->time();

  // _queued_loops may be mutated asynchronously by functions like pop_loop
  // Thus a mutex will help prevent issues
  std::unique_lock<std::mutex> lock(_mutex);

  int insertPos = 0;
  for (auto it = _queued_loops.begin(); it != _queued_loops.end(); it++)
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
  lock.unlock();

  Q_EMIT configChanged();
}

void RmfPanel::pop_delivery()
{
  auto msg = _queued_deliveries.begin()->second;

  // _queued_deliveries may be mutated asynchronously by functions like queue_delivery
  // Thus a mutex will help prevent issues
  std::lock_guard<std::mutex> lock(_mutex);

  _delivery_pub->publish(msg);
  _queued_deliveries.erase(_queued_deliveries.begin());
}

void RmfPanel::pop_loop()
{
  auto msg = _queued_loops.begin()->second;

  // _pop_loop may be mutated asynchronously by functions like queue_loop
  // Thus a mutex will help prevent issues
  std::lock_guard<std::mutex> lock(_mutex);

  _loop_pub->publish(msg);
  _queued_loops.erase(_queued_loops.begin());
}

void RmfPanel::delete_plan_item()
{
  if (_plan_list_data.count() == 0)
  {
    return;
  }

  int idx = _plan_list_view->currentIndex().row();

  if (idx == -1)
  {
    RCLCPP_INFO(_node->get_logger(), "You have to select a task to delete!");
    return;
  }

  // Data structures can possibly be mutated and we should use a lock
  std::unique_lock<std::mutex> lock(_mutex);

  // These index the offset into the respective vectors we are aiming to delete
  int d_idx = 0;
  int l_idx = 0;
  bool is_loop = true;
  std::cout << "Index" << idx << std::endl;
  for (int i = 0; i <= idx; i++) // We need to iterate at least once
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

  lock.unlock();

  Q_EMIT configChanged();
}

// Updates
void RmfPanel::update_fleet_selector()
{
  bool new_fleet_found =
    (_fleet_selector->count() != (int)_map_fleet_to_robots.size());
  if (new_fleet_found)
  {
    RCLCPP_INFO(_node->get_logger(), "New Fleet found, refreshing...");
    _fleet_selector->clear();
    for (auto it : _map_fleet_to_robots)
    {
      _fleet_selector->addItem(QString(it.first.c_str()));
    }
  }
}

void RmfPanel::open_load_file_dialog()
{
  RCLCPP_INFO(
    _node->get_logger(), "Opening Load File Dialog for Yaml Action Plan..");
  _load_file_dialog->show();
}

void RmfPanel::load_plan_from_file(const QString& path_name)
{
  RCLCPP_INFO(_node->get_logger(), "Loading File...");
  _load_file_dialog->hide();
  auto action_plan = parse_yaml_config(path_name.toStdString());
  if (!action_plan)
  {
    RCLCPP_ERROR(
      _node->get_logger(),
      "Something went wrong with loading the action plan file.");
  }
  else
  {
    RCLCPP_INFO(_node->get_logger(), "Action File successfully loaded.");

    std::unique_lock<std::mutex> lock(_mutex);
    _queued_deliveries = action_plan->first;
    _queued_loops = action_plan->second;
    lock.unlock();

    Q_EMIT configChanged();
  }
}

void RmfPanel::publish_emergency_signal()
{
  Bool msg = Bool();
  if (_emergency_state_checkbox->isChecked())
  {
    msg.data = true;
    _emergency_state_pub->publish(msg);
    RCLCPP_INFO(_node->get_logger(), "GIT TO DA CHOPPA");
  }
  else
  {
    msg.data = false;
    _emergency_state_pub->publish(msg);
  }
}

void RmfPanel::update_start_waypoint_selector()
{
  std::string fleet_name = _fleet_selector->currentText().toStdString();
  auto graph_info = _map_fleet_to_graph_info[fleet_name];
  _start_waypoint_selector->clear();
  for (const auto& waypoint : graph_info.graph.keys())
  {
    if (!_workcells_only_checkbox->isChecked() ||
      waypoint_has_workcell(waypoint.first, graph_info))
      _start_waypoint_selector->addItem(QString(waypoint.first.c_str()));
  }
}

void RmfPanel::update_end_waypoint_selector()
{
  std::string fleet_name = _fleet_selector->currentText().toStdString();
  auto graph_info = _map_fleet_to_graph_info[fleet_name];
  _end_waypoint_selector->clear();
  for (const auto& waypoint : graph_info.graph.keys())
  {
    if (!_workcells_only_checkbox->isChecked() ||
      waypoint_has_workcell(waypoint.first, graph_info))
      _end_waypoint_selector->addItem(QString(waypoint.first.c_str()));
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

void RmfPanel::update_plan()
{
  std::lock_guard<std::mutex> lock(_mutex);

  int queued_deliveries_count = _queued_deliveries.size();
  int queued_loops_count = _queued_loops.size();
  int plan_count = _plan_list_data.size();
  bool plan_changed =
    (plan_count != queued_deliveries_count + queued_loops_count);
  bool plan_empty = (_queued_deliveries.size() + _queued_loops.size()) == 0;

  _plan_list_data = QStringList();

  if (plan_empty)
  {
  }
  else if (plan_changed)
  {
    // First clear all data

    // Iterate down _queued_deliveries and _queued_loops, appending them to the
    // Plan
    auto deliver_it = _queued_deliveries.cbegin();
    auto loop_it = _queued_loops.cbegin();
    QTime ref_time;

    // Initialize reference time
    if (loop_it == _queued_loops.cend())
    {
      ref_time = deliver_it->first;
    }
    else if (deliver_it == _queued_deliveries.cend())
    {
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

    while (deliver_it != _queued_deliveries.cend() ||
      loop_it != _queued_loops.cend())
    {
      // Iterate over deliveries and loops, appending to Plan list in
      // chronological order
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
           << "Fleet: "  + loop_it->second.robot_type + "\t"
           << "From: " + loop_it->second.start_name + "\t"
           << "To: " + loop_it->second.finish_name + "\t"
           << "Repeat: " + std::to_string(loop_it->second.num_loops) + "\t"
           << std::endl;
        _plan_list_data.append(loop_it->first.toString() +
          QString(ss.str().c_str()));
        loop_it++;
      }
      else if (add_from_del)
      {
        std::stringstream ss;
        ss << "\tDelivery\t"
           << "From: " + deliver_it->second.pickup_place_name + "\t"
           << "To: " + deliver_it->second.dropoff_place_name << std::endl;
        _plan_list_data.append(deliver_it->first.toString() +
          QString(ss.str().c_str()));
        deliver_it++;
      }
    }
  }
  for (auto i : _plan_list_data)
  {
    std::cout << i.toStdString() << std::endl;
  }
  _plan_list_model->setStringList(_plan_list_data);
}

void RmfPanel::pop_plan()
{
  if (_pause_plan_checkbox->isChecked())
  {
    // If Plan is paused, we will not execute any Plan items.
    return;
  }

  for (auto delivery_data : _queued_deliveries)
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

  for (auto loop_data : _queued_loops)
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
void RmfPanel::_fleet_state_callback(const FleetState::SharedPtr msg)
{
  // RCLCPP_INFO(_node->get_logger(), "Received FleetState!");
  bool should_update = false;
  auto fleet_name = msg->name;
  if (_map_fleet_to_robots.find(fleet_name) == _map_fleet_to_robots.end())
  {
    // Fleet is new, load parameters from parameter service
    auto graph_info = load_fleet_graph_info(fleet_name);
    if (graph_info)
    {
      // Update Fleet Graph
      _map_fleet_to_graph_info.insert(
        std::pair<std::string, GraphInfo>(fleet_name, graph_info.value()));
      should_update = true;
    }
  }

  // Update robot states locally
  for (auto robot_state : msg->robots)
  {
    _map_robot_to_state[robot_state.name] = robot_state;
    // TODO(BH): Figure out why make_shared doesn't work?
    // auto robots =
    // std::make_shared<std::vector<std::string>>(_map_fleet_to_robots[fleet_name]);
    auto robots = &_map_fleet_to_robots[fleet_name];
    if (std::find(robots->begin(), robots->end(),
      robot_state.name) == robots->end())
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
  _fleet_summary_data.append(QTime::currentTime().toString() + QString("\t") +
    QString(msg->status.c_str()));
}

} // namespace rmf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmf_rviz_plugin::RmfPanel, rviz_common::Panel)
