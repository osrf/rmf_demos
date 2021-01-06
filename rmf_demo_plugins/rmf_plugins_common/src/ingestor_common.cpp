#include <rmf_plugins_common/ingestor_common.hpp>

using namespace rmf_plugins_utils;

namespace rmf_ingestor_common {

void TeleportIngestorCommon::send_ingestor_response(uint8_t status) const
{
  auto response = make_response<IngestorResult>(
    status, sim_time, latest.request_guid, _guid);
  _result_pub->publish(*response);
}

void TeleportIngestorCommon::fleet_state_cb(FleetState::UniquePtr msg)
{
  fleet_states[msg->name] = std::move(msg);
}

void TeleportIngestorCommon::ingestor_request_cb(IngestorRequest::UniquePtr msg)
{
  latest = *msg;

  if (_guid == latest.target_guid && !ingestor_filled)
  {
    const auto it = _past_request_guids.find(latest.request_guid);
    if (it != _past_request_guids.end())
    {
      if (it->second)
      {
        RCLCPP_WARN(ros_node->get_logger(),
          "Request already succeeded: [%s]", latest.request_guid.c_str());
        send_ingestor_response(IngestorResult::SUCCESS);
      }
      else
      {
        RCLCPP_WARN(ros_node->get_logger(),
          "Request already failed: [%s]", latest.request_guid.c_str());
        send_ingestor_response(IngestorResult::FAILED);
      }
      return;
    }

    ingest = true; // Mark true to ingest item next time PreUpdate() is called
  }
}

bool TeleportIngestorCommon::ingest_from_nearest_robot(
  std::function<void(FleetStateIt,
  std::vector<SimEntity>&)> fill_robot_list_cb,
  std::function<SimEntity(const std::vector<SimEntity>&,
  bool&)> find_nearest_model_cb,
  std::function<bool(const SimEntity&)> get_payload_model_cb,
  std::function<void()> transport_model_cb,
  const std::string& fleet_name)
{
  const auto fleet_state_it = fleet_states.find(fleet_name);
  if (fleet_state_it == fleet_states.end())
  {
    RCLCPP_WARN(ros_node->get_logger(),
      "No such fleet: [%s]", fleet_name.c_str());
    return false;
  }

  std::vector<SimEntity> robot_list;
  fill_robot_list_cb(fleet_state_it, robot_list);

  bool found = false;
  SimEntity robot_model = find_nearest_model_cb(robot_list, found);
  if (!found)
  {
    RCLCPP_WARN(ros_node->get_logger(),
      "No nearby robots of fleet [%s] found.", fleet_name.c_str());
    return false;
  }

  if (!get_payload_model_cb(robot_model))
  {
    RCLCPP_WARN(ros_node->get_logger(),
      "No delivery item found on the robot.");
    return false;
  }

  transport_model_cb();
  ingestor_filled = true;
  return true;
}

void TeleportIngestorCommon::on_update(
  std::function<void(FleetStateIt,
  std::vector<SimEntity>&)> fill_robot_list_cb,
  std::function<SimEntity(const std::vector<SimEntity>&,
  bool&)> find_nearest_model_cb,
  std::function<bool(const SimEntity&)> get_payload_model_cb,
  std::function<void()> transport_model_cb,
  std::function<void(void)> send_ingested_item_home_cb)
{
  // periodic ingestor status publisher
  constexpr double interval = 2.0;
  if (sim_time - last_pub_time >= interval || ingest)
  {
    last_pub_time = sim_time;
    current_state.time = simulation_now(sim_time);

    if (ingest)
    {
      current_state.mode = IngestorState::BUSY;
      current_state.request_guid_queue = {latest.request_guid};
    }
    else
    {
      current_state.mode = IngestorState::IDLE;
      current_state.request_guid_queue.clear();
    }
    _state_pub->publish(current_state);
  }

  if (ingest)
  {
    send_ingestor_response(IngestorResult::ACKNOWLEDGED);

    bool is_success = false;
    if (!ingestor_filled)
    {
      RCLCPP_INFO(ros_node->get_logger(), "Ingesting item");
      bool res = ingest_from_nearest_robot(fill_robot_list_cb,
          find_nearest_model_cb, get_payload_model_cb,
          transport_model_cb, latest.transporter_type);
      if (res)
      {
        send_ingestor_response(IngestorResult::SUCCESS);
        last_ingested_time = sim_time;
        is_success = true;
        RCLCPP_INFO(ros_node->get_logger(), "Success");
      }
      else
      {
        send_ingestor_response(IngestorResult::FAILED);
        RCLCPP_WARN(ros_node->get_logger(), "Unable to dispense item");
      }
    }
    else
    {
      RCLCPP_WARN(ros_node->get_logger(),
        "No item to ingest: [%s]", latest.request_guid);
      send_ingestor_response(IngestorResult::FAILED);
    }

    _past_request_guids.emplace(latest.request_guid, is_success);

    ingest = false;
  }

  // Periodically try to teleport ingested item back to original location
  constexpr double return_interval = 5.0;
  if (sim_time - last_ingested_time >=
    return_interval && ingestor_filled)
  {
    send_ingested_item_home_cb();
  }
}

void TeleportIngestorCommon::init_ros_node(const rclcpp::Node::SharedPtr node)
{
  ros_node = std::move(node);

  _fleet_state_sub = ros_node->create_subscription<FleetState>(
    "/fleet_states",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&TeleportIngestorCommon::fleet_state_cb, this,
    std::placeholders::_1));

  _state_pub = ros_node->create_publisher<IngestorState>(
    "/ingestor_states", 10);

  _request_sub = ros_node->create_subscription<IngestorRequest>(
    "/ingestor_requests",
    rclcpp::SystemDefaultsQoS().reliable(),
    std::bind(&TeleportIngestorCommon::ingestor_request_cb, this,
    std::placeholders::_1));

  _result_pub = ros_node->create_publisher<IngestorResult>(
    "/ingestor_results", 10);

  current_state.guid = _guid;
  current_state.mode = IngestorState::IDLE;
}

} // namespace rmf_ingestor_common
