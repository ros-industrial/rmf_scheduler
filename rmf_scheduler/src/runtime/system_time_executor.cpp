// Copyright 2023 ROS Industrial Consortium Asia Pacific
// Copyright 2022 Open Source Robotics Foundation
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


#include <chrono>
#include <mutex>

#include "rmf_scheduler/runtime/system_time_executor.hpp"
#include "rmf_scheduler/utils/system_time_utils.hpp"

namespace rmf_scheduler
{

namespace runtime
{

SystemTimeExecutor::SystemTimeExecutor()
: spinning_(false)
{
}

SystemTimeExecutor::~SystemTimeExecutor()
{
  stop();
}

void SystemTimeExecutor::add_action(
  const data::Event & event,
  Action action)
{
  std::unique_lock<std::mutex> lk(mtx_);
  eh_.add_event(event);
  actions_[event.id] = action;
  cv_.notify_all();
}

void SystemTimeExecutor::delete_action(const std::string & id)
{
  std::unique_lock<std::mutex> lk(mtx_);
  if (eh_.has_event(id)) {
    eh_.delete_event(id);
    actions_.erase(id);
    cv_.notify_all();
  }
}

void SystemTimeExecutor::spin()
{
  if (spinning_) {
    throw exception::SystemTimeExecutorException(
            "System Time Executor is already spinning");
  }

  spinning_ = true;
  while (spinning_) {
    this->_tick();
  }
}

void SystemTimeExecutor::stop()
{
  this->spinning_ = false;
  this->cv_.notify_all();
}

void SystemTimeExecutor::_tick()
{
  {
    std::unique_lock<std::mutex> lk(mtx_);

    // wait forever if there are no tasks.
    auto next = std::chrono::system_clock::time_point::max();

    std::vector<data::Event> next_events = eh_.lookup_earliest_events();
    if (!next_events.empty()) {
      next = utils::to_chrono_time_point(next_events.front().start_time);
    }

    // don't need to check for spurious wakes since the loop will exit immediately
    // if it is not yet time to run a task.
    if (cv_.wait_until(lk, next) == std::cv_status::timeout) {
      for (auto & event : next_events) {
        // Check if event is deleted
        if (eh_.has_event(event.id)) {
          pending_actions_.push_back(actions_.at(event.id));
          actions_.erase(event.id);
          eh_.delete_event(event.id);
        }
      }
    }
  }

  if (!pending_actions_.empty()) {
    for (auto & action : pending_actions_) {
      action();
    }
    pending_actions_.clear();
  }
}

}  // namespace runtime

}  // namespace rmf_scheduler
