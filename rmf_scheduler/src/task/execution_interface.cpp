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

#include "rmf_scheduler/task/execution_interface.hpp"

namespace rmf_scheduler
{

namespace task
{

void ExecutionInterface::notify_completion(
  const std::string & id,
  bool success,
  const std::string & detail)
{
  std::lock_guard lk(mtx_);
  auto itr = observers_.begin();
  while (itr != observers_.end()) {
    (*itr)->completion_callback(id, success, detail);
    ++itr;
  }
}

void ExecutionInterface::update(
  const std::string & id,
  uint64_t remaining_time)
{
  std::lock_guard lk(mtx_);
  auto itr = observers_.begin();
  while (itr != observers_.end()) {
    (*itr)->update(id, remaining_time);
    ++itr;
  }
}

void ExecutionInterface::attach(
  std::shared_ptr<ExecutionObserverBase> observer)
{
  std::lock_guard lk(mtx_);
  observers_.push_front(observer);
}

void ExecutionInterface::detach(
  std::shared_ptr<ExecutionObserverBase>/*observer*/)
{
  std::lock_guard lk(mtx_);
  // observers_.push_back(observer);
}

}  // namespace task

}  // namespace rmf_scheduler
