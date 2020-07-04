#include "ParseActionPlan.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace rmf_rviz_plugin {

using Delivery = rmf_task_msgs::msg::Delivery;
using Loop = rmf_task_msgs::msg::Loop;
using DeliveryQueue = std::vector<std::pair<QTime, Delivery>>;
using LoopQueue = std::vector<std::pair<QTime, Loop>>;
using ActionPlan = std::pair<DeliveryQueue, LoopQueue>;

rmf_utils::optional<ActionPlan> parse_yaml_config(const std::string& path_name)
{
  std::cout << "Enter ParseActionPlan: Loading YAML file" << std::endl;

  const YAML::Node action_plan_file = YAML::LoadFile(path_name);

  if (!action_plan_file)
  {
    std::cout << "Failed to find a action plan file at " << path_name <<
      std::endl;
    return rmf_utils::nullopt;
  }

  // Load and verify contents
  const YAML::Node action_plan_node = action_plan_file["action_plan"];

  if (!action_plan_node)
  {
    std::cout << "Improperly formatted yaml: missing 'action_plan'" <<
      std::endl;
  }

  //// Can be unspecified, of which we will set time to current time
  YAML::Node start_time_node = action_plan_node["start_time"];

  QTime start_time = QTime::currentTime();
  if (!start_time_node)
  {
    std::cout << "'start_time' not found. Defaulting to current time.." <<
      std::endl;
  }
  else
  {
    // Check if the time is in correct format for QTime
    start_time =
      QTime::fromString(QString(
          start_time_node.as<std::string>().c_str()), "hh:mm:ss");
    if (!start_time.isValid())
    {
      std::cout << "'start_time' is invalid. Defaulting to current time.." <<
        std::endl;
      start_time = QTime::currentTime();
    }
    else
    {
      std::cout << "Valid start time found!" << std::endl;
    }
  }

  //// Can be null
  const YAML::Node deliveries_plan = action_plan_node["actions"]["deliveries"];
  const YAML::Node loops_plan = action_plan_node["actions"]["loops"];

  if (!deliveries_plan)
  {
    std::cout << "No deliveries are found under 'deliveries'" << std::endl;
  }

  if (!loops_plan)
  {
    std::cout << "No loops are found under 'loops'" << std::endl;
  }

  std::cout << "Input checks successful." << std::endl;

  // Populate an ActionPlan
  ActionPlan _action_plan;

  // Sort contents of ActionPlan by increasing sec_from_start_time
  std::vector<std::pair<int, Delivery>> _sorted_deliveries;
  std::vector<std::pair<int, Loop>> _sorted_loops;

  // In the meantime, check contents of each input action as well
  std::cout << "Verifying Deliveries." << std::endl;
  for (YAML::Node action : deliveries_plan)
  {
    int idx = 0; // Number of clean inputs
    if (!action["sec_from_start_time"])
    {
      std::cout <<
        "'sec_from_start_time' parameter is missing from delivery " << idx <<
        std::endl;
      return rmf_utils::nullopt;
    }

    try
    {
      action["sec_from_start_time"].as<int>();
    }
    catch (const YAML::TypedBadConversion<int>&)
    {
      std::cout <<
        "'sec_from_start_time' is of an incorrect format from delivery " <<
        idx << std::endl;
      return rmf_utils::nullopt;
    }

    if (!action["task_id"])
    {
      std::cout << "'task_id' parameter is missing from delivery " << idx <<
        std::endl;
      return rmf_utils::nullopt;
    }
    if (!action["fleet"])
    {
      std::cout << "'fleet' parameter is missing from delivery " << idx <<
        std::endl;
      return rmf_utils::nullopt;
    }
    if (!action["count"])
    {
      std::cout << "'count' parameter is missing from delivery " << idx <<
        std::endl;
      return rmf_utils::nullopt;
    }
    if (!action["start"])
    {
      std::cout << "'start' parameter is missing from delivery " << idx <<
        std::endl;
      return rmf_utils::nullopt;
    }
    if (!action["end"])
    {
      std::cout << "'end' parameter is missing from delivery " << idx <<
        std::endl;
      return rmf_utils::nullopt;
    }

    std::cout << action["task_id"] << " verified." << std::endl;

    Delivery delivery = Delivery();
    delivery.task_id = action["task_id"].as<std::string>();
    delivery.pickup_place_name = action["start"].as<std::string>();
    delivery.dropoff_place_name = action["end"].as<std::string>();

    for (int i = 0; i < action["count"].as<int>(); i++)
    {
      _sorted_deliveries.emplace_back(
        action["sec_from_start_time"].as<int>(), delivery);
    }
    std::cout << action["task_id"] << " ROS2 message generated." << std::endl;
  }

  std::cout << "Verifying Loops." << std::endl;
  for (const auto& action : loops_plan)
  {
    int idx = 0; // Number of clean inputs
    if (!action["sec_from_start_time"])
    {
      std::cout << "'sec_from_start_time' parameter is missing from loop " <<
        idx << std::endl;
      return rmf_utils::nullopt;
    }
    if (!action["task_id"])
    {
      std::cout << "'task_id' parameter is missing from loop " << idx <<
        std::endl;
      return rmf_utils::nullopt;
    }
    if (!action["fleet"])
    {
      std::cout << "'fleet' parameter is missing from loop " << idx <<
        std::endl;
      return rmf_utils::nullopt;
    }
    if (!action["count"])
    {
      std::cout << "'count' parameter is missing from loop " << idx <<
        std::endl;
      return rmf_utils::nullopt;
    }
    if (!action["start"])
    {
      std::cout << "'start' parameter is missing from loop " << idx <<
        std::endl;
      return rmf_utils::nullopt;
    }
    if (!action["end"])
    {
      std::cout << "'end' parameter is missing from loop " << idx << std::endl;
      return rmf_utils::nullopt;
    }

    std::cout << action["task_id"] << " verified." << std::endl;
    Loop loop = Loop();
    loop.task_id = action["task_id"].as<std::string>();
    loop.robot_type = action["fleet"].as<std::string>();
    loop.start_name = action["start"].as<std::string>();
    loop.finish_name = action["end"].as<std::string>();
    loop.num_loops = action["count"].as<int>();
    _sorted_loops.emplace_back(action["sec_from_start_time"].as<int>(), loop);
    std::cout << action["task_id"] << " ROS2 message generated." << std::endl;
  }

  std::cout << "YAML contents verified, sorting.." << std::endl;

  std::sort(_sorted_deliveries.begin(), _sorted_deliveries.end(),
    [](auto& left, auto& right)
    {
      return left.first < right.first;
    });

  std::sort(_sorted_loops.begin(), _sorted_loops.end(),
    [](auto& left, auto& right)
    {
      return left.first < right.first;
    });

  std::cout << "Actions sorted in increasing time from start." << std::endl;

  std::cout << "Populating ActionPlan" << std::endl;

  std::cout << "Generating Deliveries." << std::endl;
  DeliveryQueue _queued_deliveries;
  QTime delivery_start_time = start_time;
  for (auto it = _sorted_deliveries.begin(); it != _sorted_deliveries.end();
    it++)
  {
    _queued_deliveries.emplace_back(delivery_start_time.addSecs(
        it->first), it->second);
  }

  std::cout << "Generating Loops." << std::endl;
  LoopQueue _queued_loops;
  QTime loops_start_time = start_time;
  for (auto it = _sorted_loops.begin(); it != _sorted_loops.end(); it++)
  {
    _queued_loops.emplace_back(loops_start_time.addSecs(it->first), it->second);
  }

  _action_plan = std::pair<DeliveryQueue, LoopQueue>(_queued_deliveries,
      _queued_loops);

  std::cout << "Action Plan complete." << std::endl;

  return _action_plan;

}
} // namespace rmf_rviz_plugin
