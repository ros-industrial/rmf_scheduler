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

#include "rmf_scheduler_ros2/scheduler_node.hpp"
#include "rmf_scheduler_ros2/scheduler_executor.hpp"
#include "rmf_scheduler_ros2/estimate_node.hpp"
#include "rmf_scheduler_ros2/runtime_node.hpp"
#include "rmf_scheduler_ros2/builder_node.hpp"
#include "rmf_scheduler_ros2/scheduler_log_handler.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>(
    "scheduler_node",
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true));

  // Register the logger to use ROS2 logging
  rmf_scheduler_ros2::register_scheduler_log_handler(node->get_name());

  // Create main node
  auto scheduler_node = rmf_scheduler_ros2::SchedulerNode::make_node(node);

  // Initialize estimation nodes
  auto estimate_node = rmf_scheduler_ros2::EstimateNode::make_node(scheduler_node);

  // Initialize runtime nodes
  auto runtime_node = rmf_scheduler_ros2::RuntimeNode::make_node(scheduler_node);

  // Initialize builder
  auto builder_node = rmf_scheduler_ros2::BuilderNode::make_node(scheduler_node);

  rmf_scheduler_ros2::SchedulerExecutor executor;
  executor.add_scheduler_node(scheduler_node);
  executor.add_node(estimate_node->node());
  executor.add_node(runtime_node->node());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
