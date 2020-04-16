#ifndef SRC_RMF_RVIZ_PLUGIN__PARSEACTIONPLAN_HPP
#define SRC_RMF_RVIZ_PLUGIN__PARSEACTIONPLAN_HPP

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>
#include <rmf_utils/optional.hpp>
#include <QTimeEdit>

namespace rmf_rviz_plugin {
namespace utils {

using Delivery = rmf_task_msgs::msg::Delivery;
using Loop = rmf_task_msgs::msg::Loop;
using DeliveryQueue = std::vector<std::pair<QTime, Delivery>>;
using LoopQueue = std::vector<std::pair<QTime, Loop>>;
using ActionPlan = std::pair<DeliveryQueue, LoopQueue>;

}

rmf_utils::optional<utils::ActionPlan> parse_yaml_config(const std::string& path_name);

} // namespace rmf_rviz_plugin
#endif
