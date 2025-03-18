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

#ifndef RMF2_SCHEDULER__SCHEDULER_HPP_
#define RMF2_SCHEDULER__SCHEDULER_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>

#include "rmf2_scheduler/storage/schedule_stream.hpp"
#include "rmf2_scheduler/data/schedule_action.hpp"
#include "rmf2_scheduler/cache/action.hpp"
#include "rmf2_scheduler/system_time_executor.hpp"
#include "rmf2_scheduler/estimator.hpp"
#include "rmf2_scheduler/optimizer.hpp"
#include "rmf2_scheduler/process_executor.hpp"
#include "rmf2_scheduler/scheduler_options.hpp"
#include "rmf2_scheduler/task_executor.hpp"

namespace rmf2_scheduler
{

class Scheduler
{
public:
  RS_SMART_PTR_DEFINITIONS(Scheduler)

  using ReadLock = std::shared_lock<std::shared_mutex>;
  using WriteLock = std::unique_lock<std::shared_mutex>;

  Scheduler(
    SchedulerOptions::Ptr options,
    storage::ScheduleStream::Ptr stream,
    SystemTimeExecutor::Ptr system_time_executor,
    Optimizer::Ptr optimizer,
    Estimator::Ptr estimator,
    ProcessExecutor::Ptr process_executor,
    TaskExecutor::Ptr task_executor
  );

  virtual ~Scheduler();

  // /// Legacy add schedule API
  // bool add_schedule(
  //   const std::vector<data::Event::Ptr> & events,
  //   const std::vector<data::Process::Ptr> & processes,
  //   const std::vector<data::Series::Ptr> & serieses,
  //   std::string & error
  // );

  // /// Legacy update schedule API
  // bool update_schedule(
  //   const std::vector<data::Event::Ptr> & events,
  //   const std::vector<data::Process::Ptr> & processes,
  //   const std::vector<data::Series::Ptr> & serieses,
  //   std::string & error
  // );

  // /// Legacy delete schedule API
  // bool delete_schedule(
  //   const std::vector<data::Event::Ptr> & events,
  //   const std::vector<data::Process::Ptr> & processes,
  //   const std::vector<data::Series::Ptr> & serieses,
  //   std::string & error
  // );

  // /// Legacy get schedule API
  // bool get_schedule_legacy(
  //   const data::Time & start_time,
  //   const data::Time & end_time,
  //   int offset,
  //   int limit,
  //   std::vector<data::Event::Ptr> & events,
  //   std::vector<data::Process::Ptr> & processes,
  //   std::vector<data::Series::Ptr> & serieses,
  //   std::string & error
  // );

  /// Get schedule API
  bool get_schedule(
    const data::Time & start_time,
    const data::Time & end_time,
    int offset,
    int limit,
    std::vector<data::Task::ConstPtr> & tasks,
    std::vector<data::Process::ConstPtr> & processes,
    std::vector<data::Series::ConstPtr> & serieses,
    std::string & error
  );

  /// Dry run operation
  bool validate(
    cache::Action::Ptr action,
    std::string & error
  ) const;

  /// Dry run operation
  bool validate(
    const data::ScheduleAction & action,
    std::string & error
  ) const;

  /// Run operation
  bool perform(
    cache::Action::Ptr action,
    std::string & error
  );

  /// Run operation
  bool perform(
    const data::ScheduleAction & action,
    std::string & error
  );

  /// Dry run batch operation
  bool validate_batch(
    const std::vector<cache::Action::Ptr> & actions,
    std::string & error
  ) const;

  /// Dry run batch operation
  bool validate_batch(
    const std::vector<data::ScheduleAction> & actions,
    std::string & error
  ) const;

  /// Run batch operation
  bool perform_batch(
    const std::vector<cache::Action::Ptr> & actions,
    std::string & error
  );

  /// Run batch operation
  bool perform_batch(
    const std::vector<data::ScheduleAction> & actions,
    std::string & error
  );

private:
  RS_DISABLE_COPY(Scheduler)

  void _tick();

  bool _refresh_schedule(std::string & error);

  SchedulerOptions::Ptr options_;

  storage::ScheduleStream::Ptr stream_;

  SystemTimeExecutor::Ptr system_time_executor_;

  Optimizer::Ptr optimizer_;

  Estimator::Ptr estimator_;

  ProcessExecutor::Ptr process_executor_;

  TaskExecutor::Ptr task_executor_;

  cache::ScheduleCache::Ptr runtime_cache_;

  cache::ScheduleCache::Ptr static_cache_;

  data::TimeWindow static_cache_time_window_;

  mutable std::shared_mutex mtx_;

  data::Time next_tick_time_;
};

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__SCHEDULER_HPP_
