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

#include "rmf2_scheduler/task_executor_manager.hpp"
#include "rmf2_scheduler/executor_data.hpp"
#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler
{

TaskExecutorManager::TaskExecutorManager()
{
}

TaskExecutorManager::~TaskExecutorManager()
{
}

void TaskExecutorManager::load(
  const std::string & task_type,
  TaskExecutor::Ptr & task_executor
)
{
  std::lock_guard lk(mtx_);
  task_executor->init(shared_from_this());
  task_executor_map_[task_type] = task_executor;
}

void TaskExecutorManager::unload(
  const std::string & task_type
)
{
  std::lock_guard lk(mtx_);
  task_executor_map_.erase(task_type);
}

bool TaskExecutorManager::is_runnable(
  const std::string & task_type
) const
{
  std::lock_guard lk(mtx_);
  return task_executor_map_.find(task_type) != task_executor_map_.end();
}

void TaskExecutorManager::on_update(
  TaskUpdateCallback update_callback
)
{
  update_callback_ = update_callback;
}

bool TaskExecutorManager::dry_run(
  const data::Task::ConstPtr & task,
  ExecutorData * task_data,
  std::string & error
)
{
  TaskExecutor::Ptr task_executor;
  {  // Lock
    std::lock_guard lk(mtx_);
    auto task_executor_itr = task_executor_map_.find(task->type);

    if (task_executor_itr == task_executor_map_.end()) {
      error = "TaskExecutorManager dry_run failed: task type [" + task->type + "] not runnable.";
      return false;
    }
    task_executor = task_executor_itr->second;
  }  // Unlock

  return task_executor->build(task, task_data, error);
}

bool TaskExecutorManager::run_async(
  const data::Task::ConstPtr & task,
  ExecutorData * task_data,
  std::future<bool> * future,
  std::string & error
)
{
  TaskExecutor::Ptr task_executor;
  std::string task_id = task->id;
  data::Task::Ptr duplicated_task = std::make_shared<data::Task>(*task);

  {  // Lock
    std::lock_guard lk(mtx_);
    auto task_executor_itr = task_executor_map_.find(task->type);

    if (task_executor_itr == task_executor_map_.end()) {
      error = "TaskExecutorManager dry_run failed: task type [" + task->type + "] not runnable.";
      return false;
    }
    task_executor = task_executor_itr->second;

    std::promise<bool> promise;
    if (future != nullptr) {
      *future = promise.get_future();
    }

    ongoing_task_sig_[task_id] = std::move(promise);
    ongoing_task_[task_id] = duplicated_task;
  }  // Unlock

  ExecutorData task_data_temp;
  if (task_data == nullptr) {
    task_data = &task_data_temp;
  }

  // Pre-mature exit
  if (!task_executor->build(duplicated_task, task_data, error)) {
    LOG_ERROR(error.c_str());
    notify_completion(
      task->id,
      false,
      nullptr
    );
    return false;
  }

  // Pre-mature exit due to fail to start
  if (!task_executor->start(task_id, task_data, error)) {
    LOG_ERROR(error.c_str());
    notify_completion(
      task->id,
      false,
      nullptr
    );
    return false;
  }

  if (update_callback_) {
    data::Time current_time(std::chrono::system_clock::now());
    duplicated_task->status = "ongoing";
    duplicated_task->planned_start_time = duplicated_task->start_time;
    duplicated_task->planned_duration = duplicated_task->duration;
    duplicated_task->start_time = current_time;
    duplicated_task->actual_start_time = current_time;
    update_callback_(duplicated_task, false);
  }

  return true;
}

void TaskExecutorManager::update(
  const std::string & id,
  const data::Duration & remaining_duration
)
{
  data::Task::Ptr task;
  {  // Lock
    std::lock_guard lk(mtx_);
    auto itr = ongoing_task_.find(id);
    if (itr == ongoing_task_.end()) {
      return;
    }
    task = itr->second;
  }  // Unlock

  bool duration_changed = false;
  data::Duration new_duration =
    data::Time(std::chrono::system_clock::now()) + remaining_duration -
    task->start_time;

  if (!task->duration.has_value() || new_duration != task->duration.value()) {
    task->duration = new_duration;
    duration_changed = true;
  }

  if (update_callback_) {
    data::Time current_time(std::chrono::system_clock::now());
    task->status = "ongoing";
    task->actual_start_time = current_time;
    update_callback_(task, duration_changed);
  }
}

void TaskExecutorManager::notify_completion(
  const std::string & id,
  bool success,
  const ExecutorData * detail
)
{
  data::Task::Ptr task;
  {  // Lock
    std::lock_guard lk(mtx_);
    auto itr = ongoing_task_.find(id);
    if (itr == ongoing_task_.end()) {
      return;
    }
    task = itr->second;

    auto sig_itr = ongoing_task_sig_.find(id);
    if (sig_itr != ongoing_task_sig_.end()) {
      sig_itr->second.set_value(true);
      // TODO(anyone): Clean up map
    }
  }  // Unlock

  if (update_callback_) {
    data::Time current_time(std::chrono::system_clock::now());
    task->status = success ? "completed" : "failed";
    task->duration = current_time - task->start_time;
    task->actual_duration =
      current_time - task->actual_start_time.value_or(task->start_time);

    if (detail != nullptr) {
      auto detail_j = detail->get_data_as_json();
      if (detail_j.has_value()) {
        task->task_details["completion_details"] = detail_j.value();
      } else {
        task->task_details["completion_details"] = detail->get_data_as_string();
      }
    }
    update_callback_(task, true);
  }
}


}  // namespace rmf2_scheduler
