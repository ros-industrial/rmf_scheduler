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

#include "rmf_scheduler/runtime_interface.hpp"

namespace rmf_scheduler
{

void RuntimeInterfaceBase::notify_completion(
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

void RuntimeInterfaceBase::update(
  const std::string & id,
  uint64_t remaining_time)
{
  std::lock_guard lk(mtx_);
  auto itr = observers_.begin();
  while (itr != observers_.end()) {
    (*itr)->completion_callback(id, remaining_time);
    ++itr;
  }
}

void RuntimeInterfaceBase::attach(
  std::shared_ptr<RuntimeObserverBase> observer)
{
  std::lock_guard lk(mtx_);
  observers_.emplace(observer);
}

void RuntimeInterfaceBase::detach(
  std::shared_ptr<RuntimeObserverBase> observer)
{
  std::lock_guard lk(mtx_);
  observers_.erase(observer);
}

}  // namespace rmf_scheduler
