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

#include "rmf_scheduler_ros2/scheduler_executor.hpp"

namespace rmf_scheduler_ros2
{

SchedulerExecutor::SchedulerExecutor(
  const rclcpp::ExecutorOptions & options,
  size_t number_of_threads,
  bool yield_before_execute,
  std::chrono::nanoseconds next_exec_timeout)
: rclcpp::executors::MultiThreadedExecutor(
    options,
    number_of_threads,
    yield_before_execute,
    next_exec_timeout)
{
}

SchedulerExecutor::~SchedulerExecutor()
{
}

void SchedulerExecutor::add_scheduler_node(
  rmf_scheduler_ros2::SchedulerNode::SharedPtr scheduler_node_ptr)
{
  scheduler_node_ptr_ = scheduler_node_ptr;
  add_node(scheduler_node_ptr_->node());
}

void SchedulerExecutor::spin()
{
  std::thread thr(
    std::bind(
      &rmf_scheduler::Scheduler::spin,
      scheduler_node_ptr_->scheduler()));
  rclcpp::executors::MultiThreadedExecutor::spin();
  scheduler_node_ptr_->scheduler()->stop();
  thr.join();
}

}  // namespace rmf_scheduler_ros2
