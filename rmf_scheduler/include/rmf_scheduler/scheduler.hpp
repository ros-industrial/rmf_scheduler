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

#ifndef RMF_SCHEDULER__SCHEDULER_HPP_
#define RMF_SCHEDULER__SCHEDULER_HPP_

#include <cstdarg>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"

#include "rmf_scheduler/exception.hpp"
#include "rmf_scheduler/dag.hpp"
#include "rmf_scheduler/series.hpp"
#include "rmf_scheduler/events_handler.hpp"
#include "rmf_scheduler/error_code.hpp"
#include "rmf_scheduler/system_time_executor.hpp"


namespace rmf_scheduler
{

class SchemaValidator;

struct Schedule
{
  struct Description
  {
    std::unordered_map<std::string, Event> events;
    std::unordered_map<std::string, DAG::Description> dependencies;
    std::unordered_map<std::string, Series::Description> series_map;
  };

  EventsHandler eh;
  std::unordered_map<std::string, DAG> dags;
  std::unordered_map<std::string, Series> series_map;
};

class Scheduler
{
public:
  using ReadLock = std::shared_lock<std::shared_mutex>;
  using WriteLock = std::unique_lock<std::shared_mutex>;

  Scheduler();
  virtual ~Scheduler();

  /// Add a complete new schedule
  /**
   * All newly added events, dependencies and series shall not exist in the cache.
   * Depdencies and series shall all be self contained,
   * and not link to existing events in the cache.
   *
   * JSON example
   * ```json
   * {
   *   "events": {
   *     "event-11082023": {
   *       ...
   *     }
   *   },
   *   "dependencies"; {
   *     ...
   *   },
   *   "series": {
   *     ...
   *   }
   * }
   * ```
   *
   * \param[in] request_json json string for the new schedule
   * \return error code
   */
  ErrorCode add_schedule(const nlohmann::json & request_json);

  /// Add a complete new schedule
  /**
   * All newly added events, dependencies and series shall not exist in the cache.
   * Depdencies and series shall all be self contained,
   * and not link to existing events in the cache.
   *
   * \param[in] schedule Schedule description data type
   * \return error code
   */
  ErrorCode add_schedule(const Schedule::Description & schedule);

  /// Update existing schedule
  /**
   * All events, dependencies and series should already exist in the cache.
   *
   * JSON example
   * ```json
   * {
   *   "events": {
   *     "event-11082023": {
   *       ...
   *     }
   *   },
   *   "dependencies"; {
   *     ...
   *   },
   *   "series": {
   *     ...
   *   }
   * }
   * ```
   *
   * \param[in] request_json json string for the new schedule
   */
  ErrorCode update_schedule(const std::string & request_json);

  ErrorCode update_schedule(const Schedule::Description & schedule);

  std::string get_schedule(
    const nlohmann::json & request_json,
    int indent = -1) const;

  Schedule::Description get_schedule(
    uint64_t start_time = 0,
    uint64_t end_time = UINT64_MAX) const;

  ErrorCode delete_schedule(const std::string & request_json);
  void delete_events(const std::vector<std::string> & event_ids);
  void delete_dependencies(const std::vector<std::string> & dependency_ids);

  void spin();

private:
  void _add_schedule(const Schedule::Description & schedule);
  void _update_schedule(const Schedule::Description & schedule);
  ErrorCode _resolve_error_code(std::function<void()>);

  // A small convenience function for converting a thread ID to a string
  std::string _thread_id();

  mutable std::shared_mutex mtx_;
  SystemTimeExecutor ste_;
  Schedule schedule_;
  std::unique_ptr<SchemaValidator> schema_validator_;
  uint64_t last_write_time_;
  std::string last_writer_;
};

/// Exception for multiple write
/**
 * This is thrown when
 *
 * During the validation of the new changes to be made to the schedule,
 * another thread has made changes to the schedule,
 * making the already validated changes by the current thread invalid.
 */
class ScheduleMultipleWriteException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  ScheduleMultipleWriteException(const char * msg, Args && ... args)
  : ExceptionTemplate(msg, std::forward<Args>(args) ...)
  {
    add_prefix("ScheduleMultipleWriteException:\n  ");
  }
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__SCHEDULER_HPP_
