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

#ifndef RMF2_SCHEDULER__TASK_EXECUTOR_MANAGER_HPP_
#define RMF2_SCHEDULER__TASK_EXECUTOR_MANAGER_HPP_

#include <future>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rmf2_scheduler/task_executor.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

class TaskExecutorManager : public TaskExecutorManagerBase
{
public:
  RS_SMART_PTR_DEFINITIONS(TaskExecutorManager)
  using TaskExecutorLoadCallback = std::function<void (const std::string &, bool)>;
  using TaskUpdateCallback = std::function<void (const data::Task::ConstPtr &, bool)>;

  TaskExecutorManager();
  virtual ~TaskExecutorManager();

  void load(
    const std::string & task_type,
    TaskExecutor::Ptr & task_executor
  );

  void unload(
    const std::string & task_type
  );

  bool is_runnable(
    const std::string & type
  ) const;

  bool dry_run(
    const data::Task::ConstPtr &,
    ExecutorData * task_data,
    std::string & error
  );

  void update(
    const std::string & id,
    const data::Duration & remaining_duration
  ) override;

  void notify_completion(
    const std::string & id,
    bool success,
    const ExecutorData * detail
  ) override;

  bool run_async(
    const data::Task::ConstPtr &,
    ExecutorData * task_data,
    std::future<bool> * future,
    std::string & error
  );

private:
  RS_DISABLE_COPY(TaskExecutorManager)

  void on_update(TaskUpdateCallback update_callback);

  mutable std::mutex mtx_;

  std::unordered_map<std::string, TaskExecutor::Ptr> task_executor_map_;
  std::unordered_map<std::string, std::promise<bool>> ongoing_task_sig_;
  std::unordered_map<std::string, data::Task::Ptr> ongoing_task_;
  TaskUpdateCallback update_callback_;

  friend class Scheduler;
};
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__TASK_EXECUTOR_MANAGER_HPP_
