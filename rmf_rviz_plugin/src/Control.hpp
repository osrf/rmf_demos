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

#include <rmf_task_msgs/msg/delivery.msg>

#include <QLineEdit>
#include <QPushButton>

#include <memory>
#include <thread>

namesapce rmf_rviz_plugin {

using Delivery = rmf_task_msgs::msg::Delivery;

class RmfPanel : public rviz_common::Panel
{
Q_OBJECT
public:
  RmfPanel(QWidget* parent = 0);
  ~RmfPanel();

  virtual void load(const rviz_common::Config& config);
  virtual void save(rviz_common::Config config) const;

public Q_SLOTS:
  void set_delivery_task_id(const QString& max);
  void set_delivery_pickup(const QString& topic);
  void set_delivery_dropoff(const QString& map_name);
  void set_delivery_robot(const QString& map_name);

protected Q_SLOTS:
  void update_delivery_task_id();
  void update_delivery_pickup();
  void update_delivery_dropoff();
  void update_delivery_robot();
  void request_delivery();

protected:
  
  void create_layout();

  QLineEdit* _delivery_task_id_editor;
  QLineEdit* _delivery_pickup_editor;
  QLineEdit* _delivery_dropoff_editor;
  QLineEdit* _delivery_robot_editor;

  QPushButton* _delivery_button;

  QString _delivery_task_id;
  QString _delivery_pickup;
  QString _delivery_dropoff;
  QString _delivery_robot;

  rclcpp::Node::SharedPtr _node;
  rclcpp::Publisher<Delivery>::SharedPtr _delivery_pub;

  bool _has_loaded = false;
  
  std::thread _thread;

};

} // namespace rmf_rviz_plugin

#endif // RMF_RVIZ__PLUGIN__SRC__CONTROL_HPP