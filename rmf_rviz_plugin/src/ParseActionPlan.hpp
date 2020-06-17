#ifndef SRC_RMF_RVIZ_PLUGIN__PARSEACTIONPLAN_HPP
#define SRC_RMF_RVIZ_PLUGIN__PARSEACTIONPLAN_HPP

#include <rmf_task_msgs/msg/delivery.hpp>
#include <rmf_task_msgs/msg/loop.hpp>
#include <rmf_utils/optional.hpp>
#include <QTimeEdit>

namespace rmf_rviz_plugin {

rmf_utils::optional<
  std::pair<
    std::vector<std::pair<QTime, rmf_task_msgs::msg::Delivery>>,
    std::vector<std::pair<QTime, rmf_task_msgs::msg::Loop>>
  >> parse_yaml_config(const std::string& path_name);

} // namespace rmf_rviz_plugin
#endif
