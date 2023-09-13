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

#ifndef RMF_SCHEDULER_ROS2__SCHEDULER_EXECUTOR_HPP_
#define RMF_SCHEDULER_ROS2__SCHEDULER_EXECUTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rmf_scheduler_ros2/scheduler_node.hpp"

namespace rmf_scheduler_ros2
{

class SchedulerExecutor : public rclcpp::executors::MultiThreadedExecutor
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SchedulerExecutor)

  RCLCPP_PUBLIC
  explicit SchedulerExecutor(
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions(),
    size_t number_of_threads = 0,
    bool yield_before_execute = false,
    std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));


  RCLCPP_PUBLIC
  virtual ~SchedulerExecutor();

  virtual void add_scheduler_node(
    rmf_scheduler_ros2::SchedulerNode::SharedPtr scheduler_node_ptr);

  /**
   * \sa rclcpp::Executor:spin() for more details
   * \throws std::runtime_error when spin() called while already spinning
   */
  RCLCPP_PUBLIC
  void
  spin() override;

private:
  RCLCPP_DISABLE_COPY(SchedulerExecutor)

  rmf_scheduler_ros2::SchedulerNode::SharedPtr scheduler_node_ptr_;
};

}  // namespace rmf_scheduler_ros2

#endif  // RMF_SCHEDULER_ROS2__SCHEDULER_EXECUTOR_HPP_
