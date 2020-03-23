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

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QGridLayout>

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

RmfPanel::RmfPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  create_layout();
}

RmfPanel::~RmfPanel()
{
  if (_has_loaded)
  {
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

} // namespace rmf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmf_rviz_plugin::RmfPanel, rviz_common::Panel)
