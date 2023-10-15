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

#include "rmf_scheduler_ros2/builder_node.hpp"

namespace rmf_scheduler_ros2
{

BuilderNode::BuilderNode()
{
}

BuilderNode::SharedPtr BuilderNode::make_node(
  SchedulerNode::SharedPtr scheduler_node)
{
  BuilderNode::SharedPtr builder_node(new BuilderNode());

  std::string ns = scheduler_node->node()->get_namespace();
  std::string name = scheduler_node->node()->get_name();
  name += "_builder_client";
  builder_node->_node = rclcpp::Node::make_shared(
    name, ns,
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
  );

  builder_node->_scheduler = scheduler_node->scheduler();

  // Load plugins through parameters
  auto result = builder_node->_node->list_parameters({}, 2);
  for (auto & plugin_name : result.prefixes) {
    auto plugin_type = builder_node->_node->get_parameter(plugin_name + ".type").as_string();
    auto supported_tasks =
      builder_node->_node->get_parameter(plugin_name + ".supported_tasks").as_string_array();
    RCLCPP_INFO(
      builder_node->_node->get_logger(),
      "Builder plugin found: %s, type: %s",
      plugin_name.c_str(), plugin_type.c_str());
    builder_node->load_interface(plugin_name, plugin_type, supported_tasks);
  }

  return builder_node;
}

void BuilderNode::load_interface(
  const std::string & name,
  const std::string & interface,
  const std::vector<std::string> & task_types)
{
  _scheduler->load_builder_interface(
    _node,
    name,
    interface,
    task_types);
}

void BuilderNode::unload_interface(
  const std::string & name)
{
  _scheduler->unload_builder_interface(name);
}

rclcpp::Node::SharedPtr BuilderNode::node() const
{
  return _node;
}


}  // namespace rmf_scheduler_ros2
