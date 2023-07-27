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

#ifndef RMF_SCHEDULER__RUNTIME_INTERFACE_HPP_
#define RMF_SCHEDULER__RUNTIME_INTERFACE_HPP_

#include <memory>
#include <mutex>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

#include "rmf_scheduler/error_code.hpp"

namespace rmf_scheduler
{

class RuntimeObserverBase
{
public:
  virtual ~RuntimeObserverBase() {}
  virtual void completion_callback(
    const std::string & id,
    bool success,
    const std::string & detail = "") = 0;

  virtual void update(
    const std::string & id,
    uint64_t remaining_time) = 0;
};

/// Template for designing a runtime handler for a task
class RuntimeInterfaceBase
{
public:
  virtual ~RuntimeInterfaceBase() {}
  virtual void init(
    const std::string & name,
    const std::string & ns = "") = 0;

  virtual void start(
    const std::string & id,
    const std::string & event_details) = 0;

  virtual void pause(const std::string & id) = 0;

  virtual void resume(const std::string & id) = 0;

  virtual void cancel(const std::string & id) = 0;

  virtual const std::vector<std::string> & support_task_types() const = 0;

  void update(
    const std::string & id,
    uint64_t remaining_time);

  void notify_completion(
    const std::string & id,
    bool success,
    const std::string & detail);

  void attach(std::shared_ptr<RuntimeObserverBase> observer);

  void detach(std::shared_ptr<RuntimeObserverBase> observer);

private:
  std::mutex mtx_;
  std::unordered_set<std::shared_ptr<RuntimeObserverBase>> observers_;
};


}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__RUNTIME_INTERFACE_HPP_
