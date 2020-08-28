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

#ifndef RMF_RVIZ__PLUGIN__SRC__CONTROL_HPP
#define RMF_RVIZ__PLUGIN__SRC__CONTROL_HPP

#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_default_plugins/tools/point/point_tool.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_request.hpp>
#include <rmf_dispenser_msgs/msg/dispenser_state.hpp>
#include <rmf_door_msgs/msg/door_mode.hpp>
#include <rmf_door_msgs/msg/door_request.hpp>
#include <rmf_door_msgs/msg/door_state.hpp>
#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_fleet_msgs/msg/mode_request.hpp>
#include <rmf_fleet_msgs/msg/path_request.hpp>
#include <rmf_fleet_msgs/msg/robot_mode.hpp>
#include <rmf_fleet_msgs/msg/robot_state.hpp>
#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>
#include <rmf_task_msgs/msg/tasks.hpp>
#include <std_msgs/msg/bool.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QFileDialog>
#include <QLineEdit>
#include <QListView>
#include <QPushButton>
#include <QSpinBox>
#include <QStringListModel>
#include <QTextEdit>
#include <QTimeEdit>
#include <QTimer>

#include <memory>
#include <mutex>
#include <thread>

#include "ParseGraph.hpp"
#include "ParseActionPlan.hpp"

namespace rmf_rviz_plugin {

class RmfPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  RmfPanel(QWidget* parent = 0);
  ~RmfPanel();

  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void queue_delivery();
  void queue_loop();
  void pop_delivery();
  void pop_plan();
  void pop_loop();
  void delete_plan_item();
  void load_plan_from_file(const QString& file);
  void open_load_file_dialog();
  void publish_emergency_signal();

  void publish_emergency();
  void update_emergency();

protected Q_SLOTS:
  void update_fleet_selector();
  void update_start_waypoint_selector();
  void update_end_waypoint_selector();
  void update_time_selector();
  void update_task_summary_list();
  void update_plan();

protected:
  void create_layout();
  void initialize_publishers();
  void initialize_subscribers();
  void initialize_state_record();
  void initialize_qt_connections();
  void initialize_models();

  // Options - For configuring certain behaviors in the GUI
  QCheckBox* _update_time_checkbox; // If checked, update time in _time_selector
  QCheckBox* _pause_plan_checkbox;  // If checked, pause plan running
  QCheckBox* _workcells_only_checkbox; // If checked, only, workcell waypoints
                                       // will be available for selection
  QCheckBox* _emergency_state_checkbox; // If checked, all robots to the choppa

  // Selectors - For targeting agents to accomplish goals
  QComboBox* _fleet_selector;
  QComboBox* _start_waypoint_selector;
  QComboBox* _end_waypoint_selector;
  QTimeEdit* _time_selector;

  // Status - For visualizing important inforation on the selected agent
  QListView* _fleet_summary_view; // Displays task summaries from rmf_core
  QStringListModel* _fleet_summary_model;
  QStringList _fleet_summary_data;

  // Plans - For visualizing and planning future actions
  QListView* _plan_list_view; // Displays [action] by [fleet] at [time]
  QPushButton* _load_action_plan_button;
  QPushButton* _delete_plan_item_button;

  QStringListModel* _plan_list_model;
  QStringList _plan_list_data;
  std::vector<std::pair<QTime,
    rmf_task_msgs::msg::Delivery>> _queued_deliveries;
  std::vector<std::pair<QTime, rmf_task_msgs::msg::Loop>> _queued_loops;

  // Actions - For queuing commands in Plan
  QLineEdit* _pickup_dispenser_editor;
  QString _pickup_dispenser;
  QLineEdit* _dropoff_ingestor_editor;
  QString _dropoff_ingestor;
  QPushButton* _send_delivery_button;

  QSpinBox* _repeat_count_selector; // Number of loops in Loop task
  QPushButton* _send_loop_button;

  // QTimer to update fields
  QTimer* _update_timer;

  bool _has_loaded = false;

  std::thread _thread;
  std::mutex _mutex;
  rclcpp::Node::SharedPtr _node;

  // File Dialog to load yaml config files for planned actions
  QFileDialog* _load_file_dialog;

private:
  // ROS2 Plumbing
  rclcpp::Subscription<rmf_fleet_msgs::msg::FleetState>::SharedPtr
    _fleet_state_sub;
  rclcpp::Subscription<rmf_task_msgs::msg::TaskSummary>::SharedPtr
    _task_summary_sub;

  rclcpp::Publisher<rmf_task_msgs::msg::Delivery>::SharedPtr _delivery_pub;
  rclcpp::Publisher<rmf_task_msgs::msg::Loop>::SharedPtr _loop_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _emergency_state_pub;

  // Book Keeping
  std::unordered_map<std::string, std::vector<std::string>>
  _map_fleet_to_robots;
  std::unordered_map<std::string, GraphInfo> _map_fleet_to_graph_info;
  std::unordered_map<std::string,
    rmf_fleet_msgs::msg::RobotState> _map_robot_to_state;

  // Misc Functions
  rmf_utils::optional<GraphInfo>
  load_fleet_graph_info(std::string fleet_name) const;
  std::string generate_task_uuid(const int len);
  bool waypoint_has_workcell(const std::string waypoint_name,
    const GraphInfo& graph_info);

  // ROS2 callbacks
  void _fleet_state_callback(
    const rmf_fleet_msgs::msg::FleetState::SharedPtr msg);
  void _task_summary_callback(
    const rmf_task_msgs::msg::TaskSummary::SharedPtr msg);
};
} // namespace rmf_rviz_plugin

#endif // RMF_RVIZ__PLUGIN__SRC__CONTROL_HPP
