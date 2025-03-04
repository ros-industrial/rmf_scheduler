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

#ifndef RMF_SCHEDULER_ROS2__SCHEDULER_NODE_HPP_
#define RMF_SCHEDULER_ROS2__SCHEDULER_NODE_HPP_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <rmf_scheduler/scheduler.hpp>
#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>
#include <std_msgs/msg/string.hpp>

namespace rmf_scheduler_ros2
{

class SchedulerNode
{
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(SchedulerNode)
  static SchedulerNode::SharedPtr make_node(
    rclcpp::Node::SharedPtr scheduler_node,
    const rmf_scheduler::SchedulerOptions::DynamicChargerMap & dynamic_charger_map,
    const rmf_scheduler::SchedulerOptions::FixedChargerMap & fixed_charger_map
  );

  static SchedulerNode::SharedPtr make_node(
    rclcpp::Node::SharedPtr scheduler_node);


  rclcpp::Node::SharedPtr node() const;

  std::shared_ptr<rmf_scheduler::Scheduler> scheduler();

  std::shared_ptr<const rmf_scheduler::Scheduler> scheduler_const() const;

private:
  rclcpp::Node::SharedPtr _node;

  std::shared_ptr<rmf_scheduler::Scheduler> _scheduler;

  rclcpp::Publisher<rmf_task_msgs::msg::ApiResponse>::SharedPtr _scheduler_response_publisher;

  rclcpp::Subscription<rmf_task_msgs::msg::ApiRequest>::SharedPtr _scheduler_request_subscriber;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _test_deconflict_sub;
  SchedulerNode();

  void schedule_request_cb(const rmf_task_msgs::msg::ApiRequest & msg);
  void test_deconflict_cb(const std_msgs::msg::String & msg);
};

}  // namespace rmf_scheduler_ros2

#endif  // RMF_SCHEDULER_ROS2__SCHEDULER_NODE_HPP_
