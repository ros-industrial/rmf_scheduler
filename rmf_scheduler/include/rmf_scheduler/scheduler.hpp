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

#include "rclcpp/rclcpp.hpp"

#include "rmf_scheduler/exception.hpp"
#include "rmf_scheduler/schedule.hpp"
#include "rmf_scheduler/series.hpp"
#include "rmf_scheduler/events_handler.hpp"
#include "rmf_scheduler/error_code.hpp"
#include "rmf_scheduler/system_time_executor.hpp"
#include "rmf_scheduler/update_event_time.hpp"


namespace rmf_scheduler
{

class SchemaValidator;

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
   * \throw ExceptionTemplate exception defined
   */
  void add_schedule(const Schedule::Description & schedule);

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
  ErrorCode update_schedule(const nlohmann::json & request_json);

  void update_schedule(const Schedule::Description & schedule);

  ErrorCode update_event_time(const nlohmann::json & request_json);

  ErrorCode update_event_time(const UpdateEventTime & update);

  nlohmann::json get_schedule(const nlohmann::json & request_json) const;

  Schedule::Description get_schedule(
    uint64_t start_time = 0,
    uint64_t end_time = UINT64_MAX) const;

  ErrorCode delete_schedule(const nlohmann::json & request_json);
  void delete_schedule(
    const std::vector<std::string> & event_ids,
    const std::vector<std::string> & dependency_ids,
    const std::vector<std::string> & series_ids);

  const Schedule & get_schedule_handler_const() const;
  Schedule & get_schedule_handler();

  // Runtime related functions
  void load_runtime_interface(
    const std::string & interface,
    const std::string & name,
    const std::string & ns = "");

  void unload_runtime_interface(
    const std::string & name,
    const std::string & ns = "");

  void spin();

  rclcpp::Executor::SharedPtr get_ros_executor() const;

private:
  // A small convenience function for converting a thread ID to a string
  std::string _thread_id();

  void add_events_to_runtime(uint64_t start_time, uint64_t end_time);

  mutable std::shared_mutex mtx_;
  Schedule schedule_;
  std::unique_ptr<SchemaValidator> schema_validator_;
  uint64_t last_write_time_;
  std::string last_writer_;

  // ROS related
  rclcpp::Executor::SharedPtr ros_executor_;
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
