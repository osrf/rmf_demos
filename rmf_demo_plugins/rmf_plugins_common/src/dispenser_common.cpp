#include <memory>

#include <rmf_plugins_common/dispenser_common.hpp>
#include <rmf_plugins_common/utils.hpp>

namespace rmf_dispenser_common {

void TeleportDispenserCommon::send_dispenser_response(uint8_t status) const
{
  auto response = rmf_plugins_utils::make_response<DispenserResult>(
    status, sim_time, latest.request_guid, guid);
  _result_pub->publish(*response);
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

void TeleportDispenserCommon::try_refill_dispenser(
  std::function<bool(void)> check_filled_cb)
{
  constexpr double interval = 2.0;
  if (sim_time - last_pub_time >= interval)
  {
    // Occasionally check to see if dispensed item has been returned to it
    if (!dispenser_filled && item_en_found && check_filled_cb())
    {
      dispenser_filled = true;
    }
  }
}

void TeleportDispenserCommon::on_update(
  std::function<bool(const std::string&)> dispense_onto_robot_cb,
  std::function<bool(void)> check_filled_cb)
{
  // `_dispense` is set to true if the dispenser plugin node has received a valid DispenserRequest
  if (dispense)
  {
    send_dispenser_response(DispenserResult::ACKNOWLEDGED);

    if (dispenser_filled)
    {
      RCLCPP_INFO(ros_node->get_logger(), "Dispensing item");
      bool res = dispense_onto_robot_cb(latest.transporter_type);
      if (res)
      {
        send_dispenser_response(DispenserResult::SUCCESS);
        RCLCPP_INFO(ros_node->get_logger(), "Success");
      }
      else
      {
        send_dispenser_response(DispenserResult::FAILED);
        RCLCPP_WARN(ros_node->get_logger(), "Unable to dispense item");
      }
    }
    else
    {
      RCLCPP_WARN(ros_node->get_logger(),
        "No item to dispense: [%s]", latest.request_guid);
      send_dispenser_response(DispenserResult::FAILED);
    }
    dispense = false;
  }

  constexpr double interval = 2.0;
  if (sim_time - last_pub_time >= interval)
  {
    last_pub_time = sim_time;
    const auto now = rmf_plugins_utils::simulation_now(sim_time);

    current_state.time = now;
    current_state.mode = DispenserState::IDLE;
    _state_pub->publish(current_state);
  }

  try_refill_dispenser(check_filled_cb);
}

void TeleportDispenserCommon::init_ros_node(const rclcpp::Node::SharedPtr node)
{
  ros_node = std::move(node);

  _fleet_state_sub = ros_node->create_subscription<FleetState>(
    "/fleet_states",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&TeleportDispenserCommon::fleet_state_cb, this,
    std::placeholders::_1));

  _state_pub = ros_node->create_publisher<DispenserState>(
    "/dispenser_states", 10);

  _request_sub = ros_node->create_subscription<DispenserRequest>(
    "/dispenser_requests",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&TeleportDispenserCommon::dispenser_request_cb, this,
    std::placeholders::_1));

  _result_pub = ros_node->create_publisher<DispenserResult>(
    "/dispenser_results", 10);

  current_state.guid = guid;
  current_state.mode = DispenserState::IDLE;
}

} // namespace rmf_dispenser_common