#include "ParseActionPlan.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace rmf_rviz_plugin {

rmf_utils::optional<ActionPlan> parse_yaml_config(std::string path_name)
{
  std::cout << "Enter ParseActionPlan: Loading YAML file" << std::endl;

  const YAML::Node action_plan_file = YAML::LoadFile(path_name);

  if (action_plan_file.IsNull())
  {
    std::cout << "Failed to find a action plan file at " << path_name << std::endl;
    return rmf_utils::nullopt;
  }

  // Load and verify contents
  const YAML::Node action_plan = action_plan_file["action_plan"];

  if (action_plan.IsNull())
  {
    std::cout << "Improperly formatted yaml: missing 'action_plan'" << std::endl;
  }

  //// Can be null, of which we will set time to current time
  const YAML::Node start_time_node = action_plan["start_time"];
  bool use_current_time = false;
  if (start_time_node.IsNull())
  {
    std::cout << "'start_time' not found. Defaulting to current time.." << std::endl;
    use_current_time = true;
  }

  //// TODO(BH): Implement proper formatting of start_time. Now use default to current time
  const std::string& start_time_string = start_time_node.as<std::string>();
  use_current_time = true;

  //// Can be null
  const YAML::Node deliveries_plan = action_plan["actions"]["deliveries"];
  const YAML::Node loops_plan = action_plan["actions"]["loops"];

  if (deliveries_plan.IsNull())
  {
    std::cout << "No deliveries are found under 'deliveries'" << std::endl;
  }

  if (loops_plan.IsNull())
  {
    std::cout << "No loops are found under 'loops'" << std::endl;
  }

  std::cout << "Input checks successful." << std::endl;

  // Populate an ActionPlan
  return rmf_utils::nullopt;
}

} // namespace rmf_rviz_plugin 
