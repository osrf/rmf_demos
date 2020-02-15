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

#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

namespace rmf_rviz_plugin {

void RmfPanel::create_layout()
{
  // Layout for delivery request
  // HLayout 1
  QHBoxLayout* layout_delivery1 = new QHBoxLayout;
  layout_delivery1->addWidget(new QLabel("Delivery Request"));
  layout_delivery1->addStretch();

  // HLayout 2
  QHBoxLayout* layout_delivery2 = new QHBoxLayout;
  layout_delivery2->addWidget(new QLabel("Task ID:"));
  _delivery_task_id_editor = new QLineEdit;
  _delivery_task_id_editor->setFixedWidth(200);
  layout_delivery2->addWidget(_delivery_task_id_editor);

  layout_delivery2->addWidget(new QLabel("Robot:"));
  _delivery_robot_editor = new QLineEdit;
  _delivery_robot_editor->setFixedWidth(200);
  layout_delivery2->addWidget(_delivery_robot_editor);
  layout_delivery2->addStretch();

  // HLayout 3
  QHBoxLayout* layout_delivery3 = new QHBoxLayout;
  layout_delivery3->addWidget(new QLabel("Pickup:"));
  _delivery_pickup_editor = new QLineEdit;
  _delivery_pickup_editor->setFixedWidth(200);
  layout_delivery3->addWidget(_delivery_pickup_editor);

  layout_delivery3->addWidget(new QLabel("Dropoff:"));
  _delivery_dropoff_editor = new QLineEdit;
  _delivery_dropoff_editor->setFixedWidth(200);
  layout_delivery3->addWidget(_delivery_dropoff_editor);
  layout_delivery3->addStretch();

  //HLayout 4
  QHBoxLayout* layout_delivery4 = new QHBoxLayout;
  _delivery_button = new QPushButton(this);
  _delivery_button->setText("Request Delivery");
  layout_delivery4->addWidget(_delivery_button);
  layout_delivery4->addStretch();

  QVBoxLayout* layout_delivery = new QVBoxLayout;
  layout_delivery->addLayout(layout_delivery1);
  layout_delivery->addLayout(layout_delivery2);
  layout_delivery->addLayout(layout_delivery3);
  layout_delivery->addLayout(layout_delivery4);

  // Layout for loop request
  // HLayout 1
  QHBoxLayout* layout1 = new QHBoxLayout;
  layout1->addWidget(new QLabel("Loop Request"));
  layout1->addStretch();

  // HLayout 2
  QHBoxLayout* layout2 = new QHBoxLayout;
  layout2->addWidget(new QLabel("Task ID:"));
  _loop_task_id_editor = new QLineEdit;
  _loop_task_id_editor->setFixedWidth(100);
  layout2->addWidget(_loop_task_id_editor);

  layout2->addWidget(new QLabel("Robot:"));
  _loop_robot_editor = new QLineEdit;
  _loop_robot_editor->setFixedWidth(100);
  layout2->addWidget(_loop_robot_editor);
  layout2->addStretch();

  // HLayout 3
  QHBoxLayout* layout3 = new QHBoxLayout;
  layout3->addWidget(new QLabel("Start:"));
  _loop_start_editor = new QLineEdit;
  _loop_start_editor->setFixedWidth(100);
  layout3->addWidget(_loop_start_editor);

  layout3->addWidget(new QLabel("Finish:"));
  _loop_finish_editor = new QLineEdit;
  _loop_finish_editor->setFixedWidth(100);
  layout3->addWidget(_loop_finish_editor);
  layout3->addStretch();

  //HLayout 4
  QHBoxLayout* layout4 = new QHBoxLayout;
  _loop_num_editor = new QLineEdit;
  _loop_num_editor->setFixedWidth(100);
  layout4->addWidget(_loop_num_editor);

  _loop_button = new QPushButton(this);
  _loop_button->setText("Request Loop");
  layout4->addWidget(_loop_button);
  layout4->addStretch();

  QVBoxLayout* layout_loop = new QVBoxLayout;
  layout_loop->addLayout(layout1);
  layout_loop->addLayout(layout2);
  layout_loop->addLayout(layout3);
  layout_loop->addLayout(layout4);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addStretch();
  layout->addLayout(layout_delivery);
  layout->addLayout(layout_loop);
  setLayout(layout);

  // Initialize text fields
  _loop_task_id_editor->setText(_loop_task_id);
  _loop_start_editor->setText(_loop_start);
  _loop_finish_editor->setText(_loop_finish);
  _loop_robot_editor->setText(_loop_robot);
  _loop_num_editor->setText(_loop_num);

  _delivery_task_id_editor->setText(_delivery_task_id);
  _delivery_pickup_editor->setText(_delivery_pickup);
  _delivery_dropoff_editor->setText(_delivery_dropoff);
  _delivery_robot_editor->setText(_delivery_robot);

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
}

RmfPanel::RmfPanel(QWidget* parent)
: rviz_common::Panel(parent),
  _delivery_task_id("task#42"),
  _delivery_robot("Magni"),
  _delivery_pickup("pantry"),
  _delivery_dropoff("hardware_2"),
  _loop_task_id("task#24"),
  _loop_robot("Magni"),
  _loop_num("1"),
  _loop_start("coe"),
  _loop_finish("cubicle_1")
{
  _node = std::make_shared<rclcpp::Node>("rmf_panel_plugin");

  _delivery_pub = _node->create_publisher<Delivery>(
    "/delivery_requests",
    rclcpp::QoS(10));

  _loop_pub = _node->create_publisher<Loop>(
    "/loop_requests",
    rclcpp::QoS(10));

  _thread = std::thread([&](){rclcpp::spin(_node);});
  
  //  create_delivery_layout();
  //  create_loop_layout();
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

void RmfPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  QString delivery_task_id;
  QString delivery_robot;
  QString delivery_pickup;
  QString delivery_dropoff;

  if (config.mapGetString("delivery_task_id", &delivery_task_id))
  {
    _delivery_task_id_editor->setText(delivery_task_id);
    update_delivery_task_id();
  }
}

void RmfPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("delivery_task_id", _delivery_task_id);
}

} // namespace rmf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmf_rviz_plugin::RmfPanel, rviz_common::Panel)
