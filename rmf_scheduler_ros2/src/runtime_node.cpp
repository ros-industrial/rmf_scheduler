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

#include "rmf_scheduler_ros2/runtime_node.hpp"

namespace rmf_scheduler_ros2
{

RuntimeNode::RuntimeNode()
{
}

RuntimeNode::SharedPtr RuntimeNode::make_node(
  SchedulerNode::SharedPtr scheduler_node)
{
  RuntimeNode::SharedPtr runtime_node(new RuntimeNode());

  std::string ns = scheduler_node->node()->get_namespace();
  std::string name = scheduler_node->node()->get_name();
  name += "_runtime_client";
  runtime_node->_node = rclcpp::Node::make_shared(
    name, ns,
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
  );

  runtime_node->_scheduler = scheduler_node->scheduler();

  return runtime_node;
}
void RuntimeNode::load_interface(
  const std::string & name,
  const std::string & interface,
  const std::vector<std::string> & task_types)
{
  _scheduler->load_runtime_interface(
    _node,
    name,
    interface,
    task_types);
}

void RuntimeNode::unload_interface(
  const std::string & name)
{
  _scheduler->unload_runtime_interface(name);
}

rclcpp::Node::SharedPtr RuntimeNode::node() const
{
  return _node;
}


}  // namespace rmf_scheduler_ros2
