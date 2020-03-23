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
#include <QPushButton>

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

  bool _has_loaded = false;
  
  std::thread _thread;
  std::mutex _mutex;

};

} // namespace rmf_rviz_plugin

#endif // RMF_RVIZ__PLUGIN__SRC__CONTROL_HPP
