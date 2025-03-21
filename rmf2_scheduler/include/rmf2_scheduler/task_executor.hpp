// Copyright 2025 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__TASK_EXECUTOR_HPP_
#define RMF2_SCHEDULER__TASK_EXECUTOR_HPP_

#include <memory>
#include <string>

#include "rmf2_scheduler/data/task.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

class ExecutorData;

class TaskExecutorManagerBase : public std::enable_shared_from_this<TaskExecutorManagerBase>
{
public:
  RS_SMART_PTR_DEFINITIONS(TaskExecutorManagerBase)

  TaskExecutorManagerBase() = default;
  virtual ~TaskExecutorManagerBase() {}

  virtual void update(
    const std::string & id,
    const data::Duration & remaining_duration
  ) = 0;

  virtual void notify_completion(
    const std::string & id,
    bool success,
    const ExecutorData * detail
  ) = 0;

private:
  friend class TaskExecutor;

  RS_DISABLE_COPY(TaskExecutorManagerBase)
};

class TaskExecutor
{
public:
  RS_SMART_PTR_DEFINITIONS(TaskExecutor)

  TaskExecutor() = default;
  virtual ~TaskExecutor() {}

  virtual bool build(
    const data::Task::ConstPtr &,
    ExecutorData * data,
    std::string & error
  ) = 0;

  virtual bool start(
    const std::string & id,
    const ExecutorData * data,
    std::string & error
  ) = 0;

  virtual bool pause(
    const std::string & /*id*/,
    std::string & error
  )
  {
    error = "Not Defined";
    return false;
  }

  virtual bool resume(
    const std::string & /*id*/,
    std::string & error
  )
  {
    error = "Not Defined";
    return false;
  }

  virtual bool cancel(
    const std::string & /*id*/,
    std::string & error
  )
  {
    error = "Not Defined";
    return false;
  }

protected:
  void update(
    const std::string & id,
    const data::Duration & remaining_duration
  )
  {
    auto manager = _acquire_sp();
    manager->update(id, remaining_duration);
  }

  void notify_completion(
    const std::string & id,
    bool success,
    const ExecutorData * detail
  )
  {
    auto manager = _acquire_sp();
    manager->notify_completion(id, success, detail);
  }

private:
  void init(const std::shared_ptr<TaskExecutorManagerBase> & manager)
  {
    manager_wp_ = manager;
  }

  std::shared_ptr<TaskExecutorManagerBase> _acquire_sp() const
  {
    std::shared_ptr<TaskExecutorManagerBase> manager;
    if (!(manager = manager_wp_.lock())) {
      throw std::runtime_error("TaskExecutorManager has expired");
    }

    return manager;
  }

  friend class TaskExecutorManager;
  std::weak_ptr<TaskExecutorManagerBase> manager_wp_;

  RS_DISABLE_COPY(TaskExecutor)
};
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__TASK_EXECUTOR_HPP_
