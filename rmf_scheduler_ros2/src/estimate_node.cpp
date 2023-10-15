// Copyright 2023 ROS Industrial Consortium Asia Pacific
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rmf_scheduler_ros2/estimate_node.hpp"

namespace rmf_scheduler_ros2
{

EstimateNode::EstimateNode()
{
}

EstimateNode::SharedPtr EstimateNode::make_node(
  SchedulerNode::SharedPtr scheduler_node)
{
  EstimateNode::SharedPtr estimate_node(new EstimateNode());

  std::string ns = scheduler_node->node()->get_namespace();
  std::string name = scheduler_node->node()->get_name();
  name += "_estimate_client";
  estimate_node->_node = rclcpp::Node::make_shared(
    name, ns,
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
  );

  estimate_node->_scheduler = scheduler_node->scheduler();

  // Load plugins through parameters
  auto result = estimate_node->_node->list_parameters({}, 2);
  for (auto & plugin_name : result.prefixes) {
    auto plugin_type = estimate_node->_node->get_parameter(plugin_name + ".type").as_string();
    auto supported_tasks =
      estimate_node->_node->get_parameter(plugin_name + ".supported_tasks").as_string_array();
    RCLCPP_INFO(
      estimate_node->_node->get_logger(),
      "Estimate plugin found: %s, type: %s",
      plugin_name.c_str(), plugin_type.c_str());
    estimate_node->load_interface(plugin_name, plugin_type, supported_tasks);
  }

  return estimate_node;
}

void EstimateNode::load_interface(
  const std::string & name,
  const std::string & interface,
  const std::vector<std::string> & task_types)
{
  _scheduler->load_estimate_interface(
    _node,
    name,
    interface,
    task_types);
}

void EstimateNode::unload_interface(
  const std::string & name)
{
  _scheduler->unload_estimate_interface(name);
}

rclcpp::Node::SharedPtr EstimateNode::node() const
{
  return _node;
}


}  // namespace rmf_scheduler_ros2
