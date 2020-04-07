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
  // Layout for delivery request
  QGridLayout* layout_delivery = new QGridLayout(this);

  // Row 1
  layout_delivery->addWidget(new QLabel("Task ID:"), 0, 0);
  _delivery_task_id_editor = new QLineEdit;
  _delivery_task_id_editor->setFixedWidth(150);
  layout_delivery->addWidget(_delivery_task_id_editor,0, 1);

  layout_delivery->addWidget(new QLabel("Robot:"), 0, 2);
  _delivery_robot_editor = new QLineEdit;
  _delivery_robot_editor->setFixedWidth(150);
  layout_delivery->addWidget(_delivery_robot_editor, 0, 3);

  // Row 2
  layout_delivery->addWidget(new QLabel("Pickup:"), 1, 0);
  _delivery_pickup_editor = new QLineEdit;
  _delivery_pickup_editor->setFixedWidth(150);
  layout_delivery->addWidget(_delivery_pickup_editor, 1, 1);

  layout_delivery->addWidget(new QLabel("Dropoff:"), 1, 2);
  _delivery_dropoff_editor = new QLineEdit;
  _delivery_dropoff_editor->setFixedWidth(150);
  layout_delivery->addWidget(_delivery_dropoff_editor, 1, 3);

  // Row 3
  _delivery_button = new QPushButton(this);
  _delivery_button->setText("Request Delivery");
  layout_delivery->addWidget(_delivery_button, 2, 1, 1, 2);

  // Creating box for delivery request
  QGroupBox* delivery_box = new QGroupBox("Delivery Request");
  delivery_box->setLayout(layout_delivery);

  // Layout for loop request
  // Row 1
  QGridLayout* layout_loop = new QGridLayout(this);
  layout_loop->addWidget(new QLabel("Task ID:"), 0, 0);
  _loop_task_id_editor = new QLineEdit;
  _loop_task_id_editor->setFixedWidth(150);
  layout_loop->addWidget(_loop_task_id_editor, 0 , 1);

  layout_loop->addWidget(new QLabel("Robot:"), 0, 2);
  _loop_robot_editor = new QLineEdit;
  _loop_robot_editor->setFixedWidth(150);
  layout_loop->addWidget(_loop_robot_editor, 0, 3);

  // Row 2
  layout_loop->addWidget(new QLabel("Start:"), 1, 0);
  _loop_start_editor = new QLineEdit;
  _loop_start_editor->setFixedWidth(150);
  layout_loop->addWidget(_loop_start_editor, 1, 1);

  layout_loop->addWidget(new QLabel("Finish:"), 1, 2);
  _loop_finish_editor = new QLineEdit;
  _loop_finish_editor->setFixedWidth(150);
  layout_loop->addWidget(_loop_finish_editor, 1, 3);

  // Row 3
  layout_loop->addWidget(new QLabel("Loops:"), 2, 0);
  _loop_num_editor = new QLineEdit;
  _loop_num_editor->setFixedWidth(100);
  layout_loop->addWidget(_loop_num_editor, 2, 1);

  // Row 4
  _loop_button = new QPushButton(this);
  _loop_button->setText("Request Loop");
  layout_loop->addWidget(_loop_button, 3, 1, 1, 2);

  // Creating box for loop request
  QGroupBox* loop_box = new QGroupBox("Loop Request");
  loop_box->setLayout(layout_loop);

  // Creating HLayout for emergency checkbox
  QHBoxLayout* emergency_layout = new QHBoxLayout;
  emergency_layout->addStretch();
  _emergency_box = new QCheckBox("Emergency Stop");
  emergency_layout->addWidget(_emergency_box);
  emergency_layout->addStretch();

  // Combining layouts into a single vertical layout
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addStretch();
  layout->addLayout(emergency_layout);
  layout->addWidget(loop_box);
  layout->addWidget(delivery_box);
  setLayout(layout);

  // Initialize text fields
  _delivery_task_id_editor->setText(_delivery_task_id);
  _delivery_pickup_editor->setText(_delivery_pickup);
  _delivery_dropoff_editor->setText(_delivery_dropoff);
  _delivery_robot_editor->setText(_delivery_robot);

  _loop_task_id_editor->setText(_loop_task_id);
  _loop_start_editor->setText(_loop_start);
  _loop_finish_editor->setText(_loop_finish);
  _loop_robot_editor->setText(_loop_robot);
  _loop_num_editor->setText(_loop_num);

  // QT signal connections
  connect(_delivery_task_id_editor,
      SIGNAL(editingFinished()), this, SLOT(update_delivery_task_id()));
  connect(_delivery_robot_editor,
      SIGNAL(editingFinished()), this, SLOT(update_delivery_robot()));
  connect(_delivery_pickup_editor,
      SIGNAL(editingFinished()), this, SLOT(update_delivery_pickup()));
  connect(_delivery_dropoff_editor,
      SIGNAL(editingFinished()), this, SLOT(update_delivery_dropoff()));
  connect(_delivery_button,
      SIGNAL(clicked()), this, SLOT(request_delivery()));

  connect(_loop_task_id_editor,
      SIGNAL(editingFinished()), this, SLOT(update_loop_task_id()));
  connect(_loop_robot_editor,
      SIGNAL(editingFinished()), this, SLOT(update_loop_robot()));
  connect(_loop_start_editor,
      SIGNAL(editingFinished()), this, SLOT(update_loop_start()));
  connect(_loop_finish_editor,
      SIGNAL(editingFinished()), this, SLOT(update_loop_finish()));
  connect(_loop_num_editor,
      SIGNAL(editingFinished()), this, SLOT(update_loop_num()));
  connect(_loop_button,
      SIGNAL(clicked()), this, SLOT(request_loop()));
  
  connect(_timer, SIGNAL(timeout()), this, SLOT(publish_emergency()));
  connect(_emergency_box, SIGNAL(stateChanged(int)), this, SLOT(update_emergency()));

}

RmfPanel::RmfPanel(QWidget* parent)
: rviz_common::Panel(parent),
  _delivery_task_id("task#42"),
  _delivery_robot("magni"),
  _delivery_pickup("pantry"),
  _delivery_dropoff("hardware_2"),
  _loop_task_id("task#24"),
  _loop_robot("magni"),
  _loop_start("coe"),
  _loop_finish("cubicle_1"),
  _loop_num("1")
{
  _node = std::make_shared<rclcpp::Node>("rmf_panel_plugin");

  _delivery_pub = _node->create_publisher<Delivery>(
    "/delivery_requests",
    rclcpp::QoS(10));

  _loop_pub = _node->create_publisher<Loop>(
    "/loop_requests",
    rclcpp::QoS(10));

  _emergency_pub = _node->create_publisher<Bool>(
    "/fire_alarm_trigger",
    rclcpp::QoS(10));

  _timer = new QTimer(this);
  _timer->start(1000);
  _emergency.data = false;

  _thread = std::thread([&](){rclcpp::spin(_node);});
  
  create_layout();
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

// Emergency Stop
void RmfPanel::publish_emergency()
{
  _emergency_pub->publish(_emergency);
}

// Update Emergency state
void RmfPanel::update_emergency()
{
  std::lock_guard<std::mutex> lock(_mutex);
  _emergency.data = _emergency_box->isChecked();
}

// Delivery set functions 
void RmfPanel::set_delivery_task_id(const QString& value)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _delivery_task_id = value;
  Q_EMIT configChanged();
}

void RmfPanel::set_delivery_pickup(const QString& value)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _delivery_pickup = value;
  Q_EMIT configChanged();
}

void RmfPanel::set_delivery_dropoff(const QString& value)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _delivery_dropoff = value;
  Q_EMIT configChanged();
}

void RmfPanel::set_delivery_robot(const QString& value)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _delivery_robot = value;
  Q_EMIT configChanged();
}

// Loop set functions
void RmfPanel::set_loop_task_id(const QString& value)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _loop_task_id = value;
  Q_EMIT configChanged();
}

void RmfPanel::set_loop_start(const QString& value)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _loop_start = value;
  Q_EMIT configChanged();
}

void RmfPanel::set_loop_finish(const QString& value)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _loop_finish = value;
  Q_EMIT configChanged();
}

void RmfPanel::set_loop_robot(const QString& value)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _loop_robot = value;
  Q_EMIT configChanged();
}

void RmfPanel::set_loop_num(const QString& value)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _loop_num = value;
  Q_EMIT configChanged();
}

// Delivery update functions 
void RmfPanel::update_delivery_task_id()
{
  set_delivery_task_id(_delivery_task_id_editor->text());
}

void RmfPanel::update_delivery_pickup()
{
  set_delivery_pickup(_delivery_pickup_editor->text());
}

void RmfPanel::update_delivery_dropoff()
{
  set_delivery_dropoff(_delivery_dropoff_editor->text());
}

void RmfPanel::update_delivery_robot()
{
  set_delivery_robot(_delivery_robot_editor->text());
}

// Loop update functions
void RmfPanel::update_loop_task_id()
{
  set_loop_task_id(_loop_task_id_editor->text());
}

void RmfPanel::update_loop_start()
{
  set_loop_start(_loop_start_editor->text());
}

void RmfPanel::update_loop_finish()
{
  set_loop_finish(_loop_finish_editor->text());
}

void RmfPanel::update_loop_robot()
{
  set_loop_robot(_loop_robot_editor->text());
}

void RmfPanel::update_loop_num()
{
  set_loop_num(_loop_num_editor->text());
}

// Publisher functions
void RmfPanel::request_delivery()
{
  Delivery delivery;

  std::lock_guard<std::mutex> lock(_mutex);
  _delivery_task_id = _delivery_task_id == "" ? "task#42" : _delivery_task_id;

  delivery.task_id = _delivery_task_id.toStdString();
  delivery.pickup_place_name = _delivery_pickup.toStdString();
  delivery.dropoff_place_name = _delivery_dropoff.toStdString();

  _delivery_pub->publish(delivery);
  RCLCPP_INFO(_node->get_logger(), "Published delivery request");
}

void RmfPanel::request_loop()
{
  Loop loop;

  std::lock_guard<std::mutex> lock(_mutex);
  _loop_task_id = _loop_task_id == "" ? "task#24" : _loop_task_id;

  loop.task_id = _loop_task_id.toStdString();
  loop.robot_type = _loop_robot.toStdString();
  loop.num_loops = _loop_num.toUInt();
  loop.start_name = _loop_start.toStdString();
  loop.finish_name = _loop_finish.toStdString();

  _loop_pub->publish(loop);
  RCLCPP_INFO(_node->get_logger(), "Published loop request");
}

// Load config
void RmfPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  QString delivery_task_id;
  QString delivery_robot;
  QString delivery_pickup;
  QString delivery_dropoff;

  QString loop_task_id;
  QString loop_robot;
  QString loop_start;
  QString loop_finish;
  QString loop_num;

  if (config.mapGetString("delivery_task_id", &delivery_task_id))
  {
    _delivery_task_id_editor->setText(delivery_task_id);
    update_delivery_task_id();
  }

  if (config.mapGetString("delivery_robot", &delivery_robot))
  {
    _delivery_robot_editor->setText(delivery_robot);
    update_delivery_robot();
  }

  if (config.mapGetString("delivery_pickup", &delivery_pickup))
  {
    _delivery_pickup_editor->setText(delivery_pickup);
    update_delivery_pickup();
  }

  if (config.mapGetString("delivery_dropoff", &delivery_dropoff))
  {
    _delivery_dropoff_editor->setText(delivery_dropoff);
    update_delivery_dropoff();
  }

  if (config.mapGetString("loop_task_id", &loop_task_id))
  {
    _loop_task_id_editor->setText(loop_task_id);
    update_loop_task_id();
  }

  if (config.mapGetString("loop_robot", &loop_robot))
  {
    _loop_robot_editor->setText(loop_robot);
    update_loop_robot();
  }

  if (config.mapGetString("loop_start", &loop_start))
  {
    _loop_start_editor->setText(loop_start);
    update_loop_start();
  }

  if (config.mapGetString("loop_finish", &loop_finish))
  {
    _loop_finish_editor->setText(loop_finish);
    update_loop_finish();
  }

  if (config.mapGetString("loop_num", &loop_num))
  {
    _loop_num_editor->setText(loop_num);
    update_loop_num();
  }
}

// Save config
void RmfPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("delivery_task_id", _delivery_task_id);
  config.mapSetValue("delivery_robot", _delivery_robot);
  config.mapSetValue("delivery_pickup", _delivery_pickup);
  config.mapSetValue("delivery_dropoff", _delivery_dropoff);

  config.mapSetValue("loop_task_id", _loop_task_id);
  config.mapSetValue("loop_robot", _loop_robot);
  config.mapSetValue("loop_start", _loop_start);
  config.mapSetValue("loop_finish", _loop_finish);
  config.mapSetValue("loop_num", _loop_num);
}

} // namespace rmf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmf_rviz_plugin::RmfPanel, rviz_common::Panel)
