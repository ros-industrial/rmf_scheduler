// Copyright 2023-2024 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__CACHE__SCHEDULE_CACHE_HPP_
#define RMF2_SCHEDULER__CACHE__SCHEDULE_CACHE_HPP_

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

#include "rmf2_scheduler/time_window_lookup.hpp"
#include "rmf2_scheduler/data/event.hpp"
#include "rmf2_scheduler/data/task.hpp"
#include "rmf2_scheduler/data/process.hpp"
#include "rmf2_scheduler/data/series.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace cache
{

class TaskHandler;
class ProcessHandler;
class SeriesHandler;

class ScheduleCache : public std::enable_shared_from_this<ScheduleCache>
{
public:
  RS_SMART_PTR_DEFINITIONS(ScheduleCache)

  ScheduleCache();

  virtual ~ScheduleCache();

  /// Get event
  data::Event::ConstPtr get_event(const std::string & event_id) const;

  /// Get all events
  std::vector<data::Event::ConstPtr> get_all_events() const;

  /// Has event
  bool has_event(const std::string & event_id) const;

  /// Lookup event based on start time and end time
  std::vector<data::Event::ConstPtr> lookup_events(
    const data::Time & start_time,
    const data::Time & end_time,
    bool soft_lower_bound = false,
    bool soft_upper_bound = true
  ) const;

  /// Number of events in the cache
  size_t event_size() const;

  // TASK
  /// Get task
  data::Task::ConstPtr get_task(const std::string & task_id) const;

  /// Get all tasks
  std::vector<data::Task::ConstPtr> get_all_tasks() const;

  /// Has task
  bool has_task(const std::string & task_id) const;

  /// Lookup task based on start time and end time
  std::vector<data::Task::ConstPtr> lookup_tasks(
    const data::Time & start_time,
    const data::Time & end_time,
    bool soft_lower_bound = false,
    bool soft_upper_bound = true
  ) const;

  /// Number of tasks in the cache
  size_t task_size() const;

  // PROCESS
  /// Get process
  data::Process::ConstPtr get_process(const std::string & process_id) const;

  /// Get all process
  std::vector<data::Process::ConstPtr> get_all_processes() const;

  /// Has process
  bool has_process(const std::string & process_id) const;

  /// Number of processes in the cache
  size_t process_size() const;

  /// Create a deep copy of the cache
  ScheduleCache::Ptr clone() const;

  // Restricted access to non const data
  class TaskRestricted;
  class ProcessRestricted;
  class SeriesRestricted;

  std::shared_ptr<TaskHandler> make_task_handler(const TaskRestricted &);

  std::shared_ptr<ProcessHandler> make_process_handler(const ProcessRestricted &);

  // SeriesMap & series_handler(const SeriesRestricted &);

private:
  RS_DISABLE_COPY(ScheduleCache)

  std::unordered_map<std::string, data::Task::Ptr> task_map;
  std::unordered_map<std::string, data::Process::Ptr> process_map;
  std::unordered_map<std::string, data::Series::Ptr> series_map;

  TimeWindowLookup time_window_lookup;

  friend class TaskHandler;
  friend class ProcessHandler;
  friend class SeriesHandler;
};

class ScheduleCache::TaskRestricted
{
private:
  TaskRestricted();
  virtual ~TaskRestricted();

  friend class EventAction;
  friend class TaskAction;
  friend class ProcessAction;
  friend class TestScheduleCache;
};

class ScheduleCache::ProcessRestricted
{
private:
  ProcessRestricted();
  virtual ~ProcessRestricted();

  friend class ProcessAction;
  friend class TestScheduleCache;
};

}  // namespace cache

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__CACHE__SCHEDULE_CACHE_HPP_
