#include <rmf_plugins_common/ingestor_common.hpp>

namespace rmf_ingestor_common {

rclcpp::Time TeleportIngestorCommon::simulation_now(double t) const
{
  const int32_t t_sec = static_cast<int32_t>(t);
  const uint32_t t_nsec =
    static_cast<uint32_t>((t-static_cast<double>(t_sec)) * 1e9);
  return rclcpp::Time{t_sec, t_nsec, RCL_ROS_TIME};
}

void TeleportIngestorCommon::send_ingestor_response(uint8_t status) const
{
  IngestorResult response;
  response.time = simulation_now(sim_time);
  response.request_guid = latest.request_guid;
  response.source_guid = _guid;
  response.status = status;
  _result_pub->publish(response);
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
          "Request already succeeded: [%s]", latest.request_guid);
        send_ingestor_response(IngestorResult::SUCCESS);
      }
      else
      {
        RCLCPP_WARN(ros_node->get_logger(),
          "Request already failed: [%s]", latest.request_guid);
        send_ingestor_response(IngestorResult::FAILED);
      }
      return;
    }

    ingest = true; // Mark true to ingest item next time PreUpdate() is called
  }
}

void TeleportIngestorCommon::on_update(){
  //if (!_load_complete)
  //  return;
  
  //_ingestor_common->sim_time =
  //  std::chrono::duration_cast<std::chrono::seconds>(info.simTime).count();
  //_ingestor_common->sim_time = _world->SimTime().Double();

  /*if (ingest)
  {
    send_ingestor_response(IngestorResult::ACKNOWLEDGED);

    if (!ingestor_filled)
    {
      RCLCPP_INFO(ros_node->get_logger(), "Ingesting item");
      ingest_from_nearest_robot(latest.transporter_type);

      send_ingestor_response(IngestorResult::SUCCESS);
      last_ingested_time = sim_time;//_world->SimTime().Double();
    }
    else
    {
      RCLCPP_WARN(ros_node->get_logger(),
        "No item to ingest: [%s]", latest.request_guid);
      send_ingestor_response(IngestorResult::FAILED);
    }
    ingest = false;
  }

  //const double t = _world->SimTime().Double();
  constexpr double interval = 2.0;
  if (sim_time - last_pub_time >= interval)
  {
    last_pub_time = sim_time;
    const auto now = simulation_now(sim_time);

    current_state.time = now;
    current_state.mode = IngestorState::IDLE;
    publish_state();
  }*/

  /*if (t - _ingestor_common->last_ingested_time >= 5.0 &&
    _ingestor_common->ingestor_filled)
  {
    send_ingested_item_home();
  }*/  
}

void TeleportIngestorCommon::init_ros_node(const rclcpp::Node::SharedPtr node)
{
  ros_node = std::move(node);

  _fleet_state_sub = ros_node->create_subscription<FleetState>(
    "/fleet_states",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&TeleportIngestorCommon::fleet_state_cb, this, std::placeholders::_1));

  _state_pub = ros_node->create_publisher<IngestorState>(
    "/ingestor_states", 10);

  _request_sub = ros_node->create_subscription<IngestorRequest>(
    "/ingestor_requests",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&TeleportIngestorCommon::ingestor_request_cb, this, std::placeholders::_1));

  _result_pub = ros_node->create_publisher<IngestorResult>(
    "/ingestor_results", 10);
}

void TeleportIngestorCommon::publish_state() const
{
  _state_pub->publish(current_state);
}

} // namespace rmf_ingestor_common