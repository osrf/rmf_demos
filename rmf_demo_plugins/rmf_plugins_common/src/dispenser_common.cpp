#include <rmf_plugins_common/dispenser_common.hpp>

namespace rmf_dispenser_common {

TeleportDispenserCommon::TeleportDispenserCommon()
{
}

rclcpp::Time TeleportDispenserCommon::simulation_now(double t) const
{
  const int32_t t_sec = static_cast<int32_t>(t);
  const uint32_t t_nsec =
    static_cast<uint32_t>((t-static_cast<double>(t_sec)) * 1e9);
  return rclcpp::Time{t_sec, t_nsec, RCL_ROS_TIME};
}

void TeleportDispenserCommon::send_dispenser_response(uint8_t status) const
{
  DispenserResult response;
  response.time = simulation_now(sim_time);
  response.request_guid = latest.request_guid;
  response.source_guid = guid;
  response.status = status;
  _result_pub->publish(response);
}

void TeleportDispenserCommon::fleet_state_cb(FleetState::UniquePtr msg)
{
  fleet_states[msg->name] = std::move(msg);
}

void TeleportDispenserCommon::dispenser_request_cb(
  DispenserRequest::UniquePtr msg)
{
  latest = *msg;

  if (guid == latest.target_guid && dispenser_filled)
  {
    const auto it = _past_request_guids.find(latest.request_guid);
    if (it != _past_request_guids.end())
    {
      if (it->second)
      {
        RCLCPP_WARN(ros_node->get_logger(),
          "Request already succeeded: [%s]", latest.request_guid);
        send_dispenser_response(DispenserResult::SUCCESS);
      }
      else
      {
        RCLCPP_WARN(ros_node->get_logger(),
          "Request already failed: [%s]", latest.request_guid);
        send_dispenser_response(DispenserResult::FAILED);
      }
      return;
    }

    dispense = true; // Mark true to dispense item next time PreUpdate() is called
  }
}

void TeleportDispenserCommon::init_ros_node(const rclcpp::Node::SharedPtr node)
{
  ros_node = std::move(node);

  _fleet_state_sub = ros_node->create_subscription<FleetState>(
    "/fleet_states",
    rclcpp::SystemDefaultsQoS(),
    [&](FleetState::UniquePtr msg)
    {
      fleet_state_cb(std::move(msg));
    });

  _state_pub = ros_node->create_publisher<DispenserState>(
    "/dispenser_states", 10);

  _request_sub = ros_node->create_subscription<DispenserRequest>(
    "/dispenser_requests",
    rclcpp::SystemDefaultsQoS(),
    [&](DispenserRequest::UniquePtr msg)
    {
      dispenser_request_cb(std::move(msg));
    });

  _result_pub = ros_node->create_publisher<DispenserResult>(
    "/dispenser_results", 10);
}

void TeleportDispenserCommon::publish_state() const
{
  _state_pub->publish(current_state);
}


} // namespace rmf_dispenser_common