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

#include "Control.hpp"

#include <QPainter>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <Qlabel>

namespace rmf_rviz_plugin {

RmfPanel::create_layout()
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
  layout_delivery4->addWidget(new QLabel("Pickup:"));
  layout_delivery4->addStretch();

  QVHoxLayout* layout_delivery = new QVBoxLayout;
  layout_delivery->addStretch();
  layout_delivery->addLayout(layout_delivery1)
  layout_delivery->addLayout(layout_delivery2)
  layout_delivery->addLayout(layout_delivery3)
  layout_delivery->addLayout(layout_delivery4)
  layout_delivery->addStretch();
}

RmfPanel::RmfPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  _node = std::make_shared<rclcpp::Node>("rmf_panel_plugin");

  _delivery_pub = _node->create_publisher<Delivery>(
    "/delivery_requests",
    rclcpp::QoS(10));

  _thread = std::thread([&](){rclcpp::spin(_node)}, this);

  // Initialize labels
  _delivery_task_id->setText("");
  _delivery_pickup->setText("pantry");
  _delivery_dropoff->setText("hardware_2");
  _delivery_robot->setText("magni");

  create_layout();

  _has_loaded = true;
}

RmfPanel::~RmfPanel()
{
  if (_has_loaded)
  {
    _thread.join()
    rclcpp::shutdown();
  }
}

void RmfPanel::set_delivery_task_id(const QString& max)
{

}
void RmfPanel::set_delivery_pickup(const QString& topic)
{

}
void RmfPanel::set_delivery_dropoff(const QString& map_name)
{

}
void RmfPanel::set_delivery_robot(const QString& map_name)
{

}

void RmfPanel::update_delivery_task_id()
{

}
void RmfPanel::update_delivery_pickup()
{

}
void RmfPanel::update_delivery_dropoff()
{

}
void RmfPanel::update_delivery_robot()
{

}
void RmfPanel::request_delivery()
{

}

} // namespace rmf_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmf_rviz_plugin::RmfPanel, rviz_common::Panel)
