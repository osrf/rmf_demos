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

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>

#include <QLineEdit>
#include <QComboBox>
#include <QTimeEdit>
#include <QPushButton>
#include <QCheckBox>
#include <QTextEdit>
#include <QListView>
#include <QSpinBox>

#include <memory>
#include <thread>
#include <mutex>

namespace rmf_rviz_plugin {

using Delivery = rmf_task_msgs::msg::Delivery;
using Loop = rmf_task_msgs::msg::Loop;

class RmfPanel : public rviz_common::Panel
{
Q_OBJECT
public:
  RmfPanel(QWidget* parent = 0);
  ~RmfPanel();

  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:

protected Q_SLOTS:

protected:
  
  void create_layout();

  // Defining GUI QT Components - Focused on Fleets
  // Selectors - For targeting agents to accomplish goals
  QComboBox* _fleet_selector; 
  QComboBox* _robot_selector;
  QComboBox* _start_waypoint_selector;
  QComboBox* _end_waypoint_selector;
  QSpinBox* _repeat_count_selector; // Determines number of times to repeat an action
  QTimeEdit* _time_selector;
  QCheckBox* _update_time_checkbox; // If checked, update time in _time_selector

  // Status - For visualizing important inforation on the selected agent
  QListView* _fleet_status_view;
  QListView* _fleet_summary_view; // Displays task summaries from rmf_core

  // Schedule - For visualizing and planning future schedule actions
  QListView* _schedule_list_view; // Displays [action] by [fleet] at [time]
  QCheckBox* _pause_schedule_checkbox; // If checked, pause schedule running
  QPushButton* _edit_schedule_item_button;
  QPushButton* _delete_schedule_item_button;

  // Actions - For queuing commands in Schedule
  QPushButton* _send_delivery_button;
  QPushButton* _send_loop_button;
  QPushButton* _pause_robot_button;
  QPushButton* _resume_robot_button;

  bool _has_loaded = false;
  
  std::thread _thread;
  std::mutex _mutex;

};

} // namespace rmf_rviz_plugin

#endif // RMF_RVIZ__PLUGIN__SRC__CONTROL_HPP
