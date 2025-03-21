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

#include "rmf2_scheduler/scheduler.hpp"
#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler
{

Scheduler::Scheduler(
  SchedulerOptions::Ptr options,
  storage::ScheduleStream::Ptr stream,
  SystemTimeExecutor::Ptr system_time_executor,
  Optimizer::Ptr optimizer,
  Estimator::Ptr estimator,
  ProcessExecutor::Ptr process_executor,
  TaskExecutorManager::Ptr task_executor_manager
)
: options_(options),
  stream_(stream),
  system_time_executor_(system_time_executor),
  optimizer_(optimizer),
  estimator_(estimator),
  process_executor_(process_executor),
  task_executor_manager_(task_executor_manager)
{
  // Error
  if (!options_) {
    throw std::runtime_error(
            "Scheduler failed: No SchedulerOptions set for Scheduler"
    );
  }

  if (!stream_) {
    throw std::runtime_error(
            "Scheduler failed: No ScheduleStream set for Scheduler"
    );
  }

  if (!system_time_executor_) {
    LOG_WARN(
      "No SystemTimeExecutor set for the scheduler, "
      "Scheduler only operates static changes."
    );
    return;
  }

  if (!optimizer_) {
    LOG_WARN(
      "No Optimizer set for Scheduler, "
      "Scheduler doesn't optimize tasks."
    );
  }

  if (!estimator_) {
    LOG_WARN(
      "No Estimator set for Scheduler, "
      "Scheduler doesn't estimate events."
    );
  }

  if (!process_executor_) {
    LOG_WARN(
      "No ProcessExecutor set for Scheduler, "
      "No doesn't execute processes."
    );
  }

  if (task_executor_manager_) {
    task_executor_manager_->on_update(
      [this](
        const data::Task::ConstPtr & task,
        bool time_changed
      ) {
        {  // Lock
          // TODO(anyone): move to private function
          WriteLock lk(mtx_);
          std::string error;

          auto task_update_action = cache::Action::create(
            data::action_type::TASK_UPDATE,
            cache::ActionPayload().task(task)
          );

          if (!task_update_action->validate(static_cache_, error)) {
            return;
          }

          task_update_action->apply();

          if (
            !stream_->write_schedule(
              static_cache_, {task_update_action->record()}, error
          ))
          {
            return;
          }
        }  // Unlock

        if (time_changed) {
          _tick();
        }
      }
    );
  } else {
    LOG_WARN(
      "No TaskExecutorManager set for Scheduler, "
      "No doesn't execute tasks."
    );
  }

  // Create static cache
  static_cache_ = cache::ScheduleCache::make_shared();
  data::Time current_time(std::chrono::system_clock::now());
  static_cache_time_window_.start = current_time - options_->static_cache_period();
  static_cache_time_window_.end = current_time + options_->static_cache_period();

  // std::string error;
  // bool result = _refresh_schedule(error);
  // if (!result) {
  //   throw std::runtime_error(error);
  // }

  // Add initial tick action
  next_tick_time_ = current_time;
  _tick();
}

Scheduler::~Scheduler()
{
}

bool Scheduler::get_schedule(
  const data::Time & start_time,
  const data::Time & end_time,
  int /*offset*/,
  int /*limit*/,
  std::vector<data::Task::ConstPtr> & tasks,
  std::vector<data::Process::ConstPtr> & processes,
  std::vector<data::Series::ConstPtr> & /*serieses*/,
  std::string & error
)
{
  // Read lock
  ReadLock lk(mtx_);

  if (start_time >= static_cache_time_window_.start &&
    end_time <= static_cache_time_window_.end)
  {
    // Get from static cache
    auto result = static_cache_->lookup_tasks(start_time, end_time);
    tasks = std::vector<data::Task::ConstPtr>(result.begin(), result.end());

    // Retrieve additional process IDs
    std::unordered_set<std::string> process_ids;
    for (auto & task : tasks) {
      if (task->process_id.has_value()) {
        process_ids.insert(*task->process_id);
      }
    }

    processes.reserve(process_ids.size());
    for (auto & process_id : process_ids) {
      auto process = static_cache_->get_process(process_id);

      // Deep copy
      processes.push_back(data::Process::make_shared(*process));
    }
  } else {
    // Retrieve from storage
    auto temp_cache = cache::ScheduleCache::make_shared();
    data::TimeWindow time_window;
    time_window.start = start_time;
    time_window.end = end_time;
    bool result = stream_->read_schedule(temp_cache, time_window, error);
    if (!result) {
      return false;
    }

    tasks = temp_cache->get_all_tasks();
    processes = temp_cache->get_all_processes();
  }
  return true;
}

bool Scheduler::validate(
  cache::Action::Ptr action,
  std::string & error
) const
{
  // Read lock
  ReadLock lk(mtx_);

  // TODO(Briancbn): Handle extended time period
  return action->validate(static_cache_, error);
}

bool Scheduler::validate(
  const data::ScheduleAction & action,
  std::string & error
) const
{
  cache::Action::Ptr cache_action = cache::Action::create(action);

  return validate(cache_action, error);
}

bool Scheduler::perform(
  cache::Action::Ptr action,
  std::string & error
)
{
  {  // Lock
    // Write lock
    WriteLock lk(mtx_);

    // Apply
    auto temp_cache = static_cache_->clone();

    // Validate
    if (!action->validate(temp_cache, error)) {
      return false;
    }

    // TODO(Briancbn): Handle extended time period

    action->apply();

    // Apply to stream
    if (
      !stream_->write_schedule(
        temp_cache,
        {action->record()},
        error
    ))
    {
      LOG_WARN(
        "Unable to apply changes to storage, revert back to last known schedule.\n%s",
        error.c_str()
      );

      // discard temp cache
      return false;
    }

    static_cache_ = temp_cache;
  }  // Unlock

  _tick();
  return true;
}

bool Scheduler::perform(
  const data::ScheduleAction & action,
  std::string & error
)
{
  cache::Action::Ptr cache_action = cache::Action::create(action);

  return perform(cache_action, error);
}

bool Scheduler::validate_batch(
  const std::vector<cache::Action::Ptr> & actions,
  std::string & error
) const
{
  // Create temporary cache
  cache::ScheduleCache::Ptr temp_cache;
  {
    // Read lock
    ReadLock lk(mtx_);

    temp_cache = static_cache_->clone();
  }

  // TODO(Briancbn): Handle extended time period

  for (auto & action : actions) {
    if (!action->validate(temp_cache, error)) {
      return false;
    }
    action->apply();
  }

  return true;
}

bool Scheduler::validate_batch(
  const std::vector<data::ScheduleAction> & actions,
  std::string & error
) const
{
  std::vector<cache::Action::Ptr> cache_actions;
  cache_actions.reserve(actions.size());

  for (auto & action : actions) {
    cache_actions.push_back(cache::Action::create(action));
  }

  return validate_batch(cache_actions, error);
}

bool Scheduler::perform_batch(
  const std::vector<cache::Action::Ptr> & actions,
  std::string & error
)
{
  {
    // Write lock
    WriteLock lk(mtx_);

    // Create temporary cache
    cache::ScheduleCache::Ptr temp_cache = static_cache_->clone();

    // TODO(Briancbn): Handle extended time period
    std::vector<data::ScheduleChangeRecord> records;
    records.reserve(actions.size());
    for (auto & action : actions) {
      if (!action->validate(temp_cache, error)) {
        return false;
      }
      action->apply();
      records.push_back(action->record());
    }

    // Apply to stream
    if (
      !stream_->write_schedule(
        temp_cache,
        records,
        error
    ))
    {
      LOG_WARN(
        "Unable to apply changes to storage, revert back to last known schedule."
      );

      // discard temp cache
      return false;
    }

    static_cache_ = temp_cache;
  }

  _tick();
  return true;
}

bool Scheduler::perform_batch(
  const std::vector<data::ScheduleAction> & actions,
  std::string & error
)
{
  std::vector<cache::Action::Ptr> cache_actions;
  cache_actions.reserve(actions.size());

  for (auto & action : actions) {
    cache_actions.push_back(cache::Action::create(action));
  }

  return perform_batch(cache_actions, error);
}

// PRIVATE
void Scheduler::_tick()
{
  // Write lock
  WriteLock lk(mtx_);  // TODO(Briancbn): Improve mutex locks

  // Add in next tick
  // First modulate the ticking period
  data::Time current_time(std::chrono::system_clock::now());
  LOG_INFO("Ticking Current Time: %s.", current_time.to_ISOtime());

  // Only tick when past the next tick time
  if (current_time >= next_tick_time_) {
    static_cache_time_window_.start = current_time - options_->static_cache_period();
    static_cache_time_window_.end = current_time + options_->static_cache_period();
    std::string error;
    bool result = _refresh_schedule(error);
    if (!result) {
      LOG_WARN("Storage unreachable, relying on local cache!!!! %s", error.c_str());
    }

    _add_next_tick();
  }

  // TODO(Briancbn): push tasks to execution
  std::vector<data::Task::ConstPtr> tasks = static_cache_->lookup_tasks(
    current_time - options_->allow_past_tasks_duration(),
    current_time + options_->runtime_tick_period()
  );

  std::unordered_set<std::string> process_ids_pushed;

  for (auto & task : tasks) {
    // Check if task is completed
    if (task->status == "completed" ||
      task->status == "failed" ||
      task->status == "cancelled" ||
      task->status == "ongoing" ||
      task->status == "paused")
    {
      LOG_DEBUG("Skipping task [%s] - %s.", task->id.c_str(), task->status.c_str());
      continue;
    }

    // Check if task belong to a process
    if (task->process_id.has_value()) {
      if (!process_executor_) {
        LOG_WARN(
          "Unable to run process [%s], ProcessExecutor not defined",
          task->process_id->c_str()
        );
        continue;
      }

      data::Process::ConstPtr process = static_cache_->get_process(*task->process_id);

      std::string error;
      if (process_ids_pushed.find(process->id) != process_ids_pushed.end()) {
        LOG_DEBUG(
          "Process [%s] already pushed"
        );
        continue;
      }

      std::vector<data::Task::ConstPtr> entry_tasks;
      std::vector<data::Task::ConstPtr> tasks;
      process->graph.for_each_node(
        [this, &tasks, &entry_tasks](const data::Node::Ptr & node) {
          auto task = static_cache_->get_task(node->id());
          auto duplicated_task = data::Task::make_shared(*task);
          if (node->inbound_edges().empty()) {
            entry_tasks.push_back(task);
          }
          tasks.push_back(duplicated_task);
        }
      );

      if (entry_tasks.empty()) {
        LOG_WARN(
          "Unable to run process [%s], no entry tasks found, process might be empty",
          process->id.c_str()
        );
        continue;
      }

      // Add process to the system time executor
      SystemTimeAction run_process_action;

      run_process_action.time = entry_tasks[0]->start_time;
      run_process_action.work = [tasks, process, this]() {
          std::string error;
          if (!process_executor_->run_async(process, tasks, error)) {
            LOG_ERROR(error.c_str());
          }
        };
      system_time_executor_->add_action(run_process_action);

      LOG_INFO(
        "Pushed process [%s] to queue for execution",
        process->id.c_str()
      );

      process_ids_pushed.emplace(process->id);
      continue;
    }

    // Run the task using task executor
    if (!task_executor_manager_->is_runnable(task->type)) {
      LOG_DEBUG(
        "Skipping task [%s] of type [%s], no task executor configured for this type.",
        task->id.c_str(),
        task->type.c_str()
      );
    }

    // Add task to the system time executor
    SystemTimeAction run_task_action;

    // duplicate the task
    run_task_action.time = task->start_time;
    run_task_action.work = [task, this]() {
        std::string error;
        if (!task_executor_manager_->run_async(task, nullptr, nullptr, error)) {
          LOG_ERROR(error.c_str());
        }
      };
    system_time_executor_->add_action(run_task_action);
  }
}

void Scheduler::_add_next_tick()
{
  next_tick_time_ = next_tick_time_ + options_->runtime_tick_period();
  LOG_INFO("Add next timer tick at time: %s.", next_tick_time_.to_ISOtime());

  // Add in next tick event
  SystemTimeAction tick_action;
  tick_action.time = next_tick_time_;
  tick_action.work = std::bind(&Scheduler::_tick, this);
  system_time_executor_->add_action(tick_action);
}

bool Scheduler::_refresh_schedule(std::string & error)
{
  auto temp_cache = cache::ScheduleCache::make_shared();

  bool result = stream_->read_schedule(
    temp_cache,
    static_cache_time_window_,
    error
  );

  if (!result) {
    error = "Unable to load schedule: " + error;
    return false;
  }

  static_cache_ = temp_cache;

  LOG_INFO(
    "Sucessfully loaded static cache within time [%s - %s]",
    std::string(static_cache_time_window_.start.to_ISOtime()).c_str(),
    std::string(static_cache_time_window_.end.to_ISOtime()).c_str()
  );

  LOG_INFO(
    "Found %lu events (%lu tasks), %lu processes",
    static_cache_->event_size(),
    static_cache_->task_size(),
    static_cache_->process_size()
  );

  return result;
}

// LockedScheduleRO
LockedScheduleRO::LockedScheduleRO(const Scheduler::ConstPtr & scheduler)
: cache_wp_(scheduler->static_cache_),
  lk_(scheduler->mtx_)
{
}

LockedScheduleRO::~LockedScheduleRO()
{
}

cache::ScheduleCache::ConstPtr LockedScheduleRO::cache() const
{
  cache::ScheduleCache::ConstPtr cache;
  if (!(cache = cache_wp_.lock())) {
    throw std::runtime_error("ScheduleCache has expired");
  }
  return cache;
}


LockedScheduleRW::LockedScheduleRW(const Scheduler::Ptr & scheduler)
: cache_wp_(scheduler->static_cache_),
  lk_(scheduler->mtx_)
{
}

LockedScheduleRW::~LockedScheduleRW()
{
}

cache::ScheduleCache::Ptr LockedScheduleRW::cache()
{
  cache::ScheduleCache::Ptr cache;
  if (!(cache = cache_wp_.lock())) {
    throw std::runtime_error("ScheduleCache has expired");
  }
  return cache;
}


}  // namespace rmf2_scheduler
