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

#ifndef RMF_PLUGINS_COMMON__INGESTOR_COMMON_HPP
#define RMF_PLUGINS_COMMON__INGESTOR_COMMON_HPP

#include <string>
#include <unordered_map>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <rmf_fleet_msgs/msg/fleet_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_state.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_result.hpp>
#include <rmf_ingestor_msgs/msg/ingestor_request.hpp>

#include <rmf_plugins_common/utils.hpp>

using namespace rmf_plugins_utils;

namespace rmf_ingestor_common {

class TeleportIngestorCommon
{
public:

  using FleetState = rmf_fleet_msgs::msg::FleetState;
  using FleetStateIt =
    std::unordered_map<std::string, FleetState::UniquePtr>::iterator;
  using IngestorState = rmf_ingestor_msgs::msg::IngestorState;
  using IngestorRequest = rmf_ingestor_msgs::msg::IngestorRequest;
  using IngestorResult = rmf_ingestor_msgs::msg::IngestorResult;

  // Ingest request params
  bool ingest = false;
  IngestorRequest latest;

  // Ingestor params
  std::string _guid;
  bool ingestor_filled = false;

  double last_pub_time = 0.0;
  double last_ingested_time = 0.0;
  double sim_time = 0.0;

  rclcpp::Node::SharedPtr ros_node;

  std::unordered_map<std::string, Eigen::Isometry3d>
  non_static_models_init_poses;
  std::unordered_map<std::string, FleetState::UniquePtr> fleet_states;
  IngestorState current_state;

  void send_ingestor_response(uint8_t status) const;
  void fleet_state_cb(FleetState::UniquePtr msg);
  void ingestor_request_cb(IngestorRequest::UniquePtr msg);
  void on_update(
    std::function<void(FleetStateIt,
    std::vector<rmf_plugins_utils::SimEntity>&)> fill_robot_list_cb,
    std::function<rmf_plugins_utils::SimEntity(
      const std::vector<rmf_plugins_utils::SimEntity>&,
      bool&)> find_nearest_model_cb,
    std::function<bool(const SimEntity&)> get_payload_model_cb,
    std::function<void()> transport_model_cb,
    std::function<void(void)> send_ingested_item_home_cb);
  void init_ros_node(const rclcpp::Node::SharedPtr node);

private:
  rclcpp::Subscription<FleetState>::SharedPtr _fleet_state_sub;
  rclcpp::Publisher<IngestorState>::SharedPtr _state_pub;
  rclcpp::Subscription<IngestorRequest>::SharedPtr _request_sub;
  rclcpp::Publisher<IngestorResult>::SharedPtr _result_pub;
  std::unordered_map<std::string, bool> _past_request_guids;

  bool ingest_from_nearest_robot(
    std::function<void(FleetStateIt,
    std::vector<rmf_plugins_utils::SimEntity>&)> fill_robot_list_cb,
    std::function<rmf_plugins_utils::SimEntity(
      const std::vector<rmf_plugins_utils::SimEntity>&,
      bool&)> find_nearest_model_cb,
    std::function<bool(const SimEntity&)> get_payload_model_cb,
    std::function<void()> transport_model_cb,
    const std::string& fleet_name);
};

// Ignition math version agnostic conversion functions
template<typename IgnQuatT>
inline void convert(const Eigen::Quaterniond& _q, IgnQuatT& quat)
{
  quat.W() = _q.w();
  quat.X() = _q.x();
  quat.Y() = _q.y();
  quat.Z() = _q.z();
}

template<typename IgnVec3T>
inline void convert(const Eigen::Vector3d& _v, IgnVec3T& vec)
{
  vec.X() = _v[0];
  vec.Y() = _v[1];
  vec.Z() = _v[2];
}

template<typename IgnVec3T>
inline Eigen::Vector3d convert_vec(const IgnVec3T& _v)
{
  return Eigen::Vector3d(_v[0], _v[1], _v[2]);
}

template<typename IgnQuatT>
inline Eigen::Quaterniond convert_quat(const IgnQuatT& _q)
{
  Eigen::Quaterniond quat;
  quat.w() = _q.W();
  quat.x() = _q.X();
  quat.y() = _q.Y();
  quat.z() = _q.Z();

  return quat;
}

template<typename IgnPoseT>
inline auto convert_to_pose(const Eigen::Isometry3d& _tf)
{
  IgnPoseT pose;
  convert(Eigen::Vector3d(_tf.translation()), pose.Pos());
  convert(Eigen::Quaterniond(_tf.linear()), pose.Rot());

  return pose;
}

template<typename IgnPoseT>
inline Eigen::Isometry3d convert_pose(const IgnPoseT& _pose)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = convert_vec(_pose.Pos());
  tf.linear() = Eigen::Matrix3d(convert_quat(_pose.Rot()));

  return tf;
}

} // namespace rmf_ingestor_common

#endif
