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

#ifndef RMF_SCHEDULER_ROS2__ESTIMATE_NODE_HPP_
#define RMF_SCHEDULER_ROS2__ESTIMATE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rmf_scheduler_ros2/scheduler_node.hpp"

namespace rmf_scheduler_ros2
{

class EstimateNode
{
public:
  // cppcheck-suppress unknownMacro
  RCLCPP_SMART_PTR_DEFINITIONS(EstimateNode)
  static EstimateNode::SharedPtr make_node(
    SchedulerNode::SharedPtr scheduler_node);

  void load_interface(
    const std::string & name,
    const std::string & interface,
    const std::vector<std::string> & task_types);

  void unload_interface(const std::string & name);

  rclcpp::Node::SharedPtr node() const;

private:
  EstimateNode();
  rclcpp::Node::SharedPtr _node;
  std::shared_ptr<rmf_scheduler::Scheduler> _scheduler;
};

}  // namespace rmf_scheduler_ros2

#endif  // RMF_SCHEDULER_ROS2__ESTIMATE_NODE_HPP_
