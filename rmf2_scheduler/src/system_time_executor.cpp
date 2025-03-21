// Copyright 2023-2025 ROS Industrial Consortium Asia Pacific
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

#include "rmf2_scheduler/system_time_executor.hpp"
#include "rmf2_scheduler/data/uuid.hpp"

namespace rmf2_scheduler
{

SystemTimeExecutor::SystemTimeExecutor()
: spinning_(false)
{
}

SystemTimeExecutor::~SystemTimeExecutor()
{
  stop();
}

std::string SystemTimeExecutor::add_action(
  const SystemTimeAction & action
)
{
  std::unique_lock<std::mutex> lk(mtx_);

  // Generate ID
  std::string id = data::gen_uuid();
  time_lookup_.emplace(action.time, id);
  actions_[id] = action;
  cv_.notify_all();
  return id;
}

void SystemTimeExecutor::delete_action(const std::string & id)
{
  std::unique_lock<std::mutex> lk(mtx_);
  auto action_itr = actions_.find(id);
  if (action_itr != actions_.end()) {
    // Remove from lookup
    _delete_time_lookup(id, action_itr->second.time);

    // Remove action
    actions_.erase(action_itr);
    cv_.notify_all();
  }
}

void SystemTimeExecutor::spin()
{
  if (spinning_) {
    throw std::runtime_error(
            "System Time Executor is already spinning"
    );
  }

  spinning_ = true;
  while (spinning_) {
    _tick();
  }
}

void SystemTimeExecutor::stop()
{
  spinning_ = false;
  cv_.notify_all();
}

std::vector<std::string>
SystemTimeExecutor::_lookup_earliest_actions() const
{
  std::vector<std::string> action_ids;
  auto itr = time_lookup_.cbegin();
  auto range = time_lookup_.equal_range(itr->first);
  for (auto i = range.first; i != range.second; ++i) {
    auto action_itr = actions_.find(i->second);
    assert(action_itr != actions_.end());
    action_ids.push_back(action_itr->first);
  }
  return action_ids;
}

void SystemTimeExecutor::_delete_time_lookup(
  const std::string & id,
  const data::Time & time
)
{
  auto time_range = time_lookup_.equal_range(time);
  for (auto itr = time_range.first; itr != time_range.second; itr++) {
    if (itr->second == id) {
      time_lookup_.erase(itr);
      break;
    }
  }
}

void SystemTimeExecutor::_tick()
{
  {
    std::unique_lock<std::mutex> lk(mtx_);

    // wait forever if there are no tasks.
    auto next_time = std::chrono::system_clock::time_point::max();

    auto next_action_ids = _lookup_earliest_actions();
    if (!next_action_ids.empty()) {
      auto next_action = actions_.at(next_action_ids[0]);
      next_time = next_action.time.to_chrono<std::chrono::system_clock>();
    }

    // don't need to check for spurious wakes since the loop will exit immediately
    // if it is not yet time to run a task.
    if (cv_.wait_until(lk, next_time) == std::cv_status::timeout) {
      for (auto & action_id : next_action_ids) {
        // Check if action is deleted
        auto action_itr = actions_.find(action_id);
        if (action_itr != actions_.end()) {
          pending_actions_.push_back(action_itr->second);
          _delete_time_lookup(action_itr->first, action_itr->second.time);
          actions_.erase(action_itr);
        }
      }
    }
  }

  if (!pending_actions_.empty()) {
    for (auto & action : pending_actions_) {
      if (action.work) {
        action.work();
      }
    }
    pending_actions_.clear();
  }
}

}  // namespace rmf2_scheduler
