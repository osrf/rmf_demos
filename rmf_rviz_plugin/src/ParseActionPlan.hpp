#ifndef SRC_RMF_RVIZ_PLUGIN__PARSEACTIONPLAN_HPP
#define SRC_RMF_RVIZ_PLUGIN__PARSEACTIONPLAN_HPP

#include <cstdint>
#include <ctime>

#include <rmf_utils/optional.hpp>
#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>

namespace rmf_rviz_plugin {

using Delivery = rmf_task_msgs::msg::Delivery;
using Loop = rmf_task_msgs::msg::Loop;

struct Action
{
  /// Seconds from ActionPlan starting time where this action will be executed
  float sec_from_start_time;
  /// Number of calls made for this Action
  uint32_t action_count;
};

struct DeliveryAction : Action
{
  Delivery delivery;
};

struct LoopAction : Action
{
  Loop loop;
};

struct ActionPlan
{
  /// intended time of execution for this ActionPlan. 
  /// If this is in the past, then the default is to treat this field as the time of execution.
  time_t start_time;

  // Contains a vector of actions in chronological order
  std::vector<Action> actions;
};

rmf_utils::optional<ActionPlan> parse_yaml_config(std::string path_name);

} // namespace rmf_rviz_plugin
#endif
