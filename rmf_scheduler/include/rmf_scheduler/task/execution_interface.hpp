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

#ifndef RMF_SCHEDULER__TASK__EXECUTION_INTERFACE_HPP_
#define RMF_SCHEDULER__TASK__EXECUTION_INTERFACE_HPP_

#include <memory>
#include <mutex>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>

#include "nlohmann/json.hpp"
#include "rmf_scheduler/error_code.hpp"
#include "rmf_scheduler/task/task_plugin.hpp"

namespace rmf_scheduler
{

namespace task
{

class ExecutionObserverBase
{
public:
  virtual ~ExecutionObserverBase() {}
  virtual void completion_callback(
    const std::string & id,
    bool success,
    const std::string & detail = "") = 0;

  virtual void update(
    const std::string & id,
    uint64_t remaining_time) = 0;
};

/// Template for designing a runtime handler for a task
class ExecutionInterface : public TaskPluginBase
{
public:
  virtual ~ExecutionInterface() {}
  void init(const std::shared_ptr<void> &) override {}


  virtual void start(
    const std::string & id,
    const nlohmann::json & event_details) = 0;

  virtual void pause(const std::string & id) = 0;

  virtual void resume(const std::string & id) = 0;

  virtual void cancel(const std::string & id) = 0;

  void update(
    const std::string & id,
    uint64_t remaining_time);

  void notify_completion(
    const std::string & id,
    bool success,
    const std::string & detail);

  void attach(std::shared_ptr<ExecutionObserverBase> observer);

  void detach(std::shared_ptr<ExecutionObserverBase> observer);

private:
  std::mutex mtx_;
  std::unordered_set<std::shared_ptr<ExecutionObserverBase>> observers_;
};

}  // namespace task

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__TASK__EXECUTION_INTERFACE_HPP_
