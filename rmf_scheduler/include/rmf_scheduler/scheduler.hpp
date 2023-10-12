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
#include <unordered_set>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"

#include "rmf_scheduler/exception.hpp"
#include "rmf_scheduler/error_code.hpp"
#include "rmf_scheduler/data/schedule.hpp"
#include "rmf_scheduler/scheduler_options.hpp"
#include "rmf_scheduler/task/builder.hpp"
#include "rmf_scheduler/task/estimator.hpp"
#include "rmf_scheduler/task/executor.hpp"
#include "rmf_scheduler/runtime/system_time_executor.hpp"
#include "rmf_scheduler/runtime/dag_executor.hpp"


namespace rmf_scheduler
{

class SchemaValidator;

class Scheduler
{
public:
  using ReadLock = std::shared_lock<std::shared_mutex>;
  using WriteLock = std::unique_lock<std::shared_mutex>;

  Scheduler();
  explicit Scheduler(const SchedulerOptions & options);
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
  ErrorCode handle_add_schedule(const nlohmann::json & request_json);

  /// Add a complete new schedule
  /**
   * All newly added events, dependencies and series shall not exist in the cache.
   * Depdencies and series shall all be self contained,
   * and not link to existing events in the cache.
   *
   * \param[in] schedule Schedule description data type
   * \throw ExceptionTemplate exception defined
   */
  void add_schedule(const data::Schedule::Description & schedule);

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
  ErrorCode handle_update_schedule(const nlohmann::json & request_json);

  void update_schedule(const data::Schedule::Description & schedule);

  ErrorCode handle_update_event_time(const nlohmann::json & request_json);

  ErrorCode handle_update_series(const nlohmann::json & request_json);

  void update_series(
    const std::unordered_map<std::string, data::Series::Update> & series_update);

  nlohmann::json handle_get_schedule(const nlohmann::json & request_json);

  data::Schedule::Description get_schedule(
    uint64_t start_time = 0,
    uint64_t end_time = UINT64_MAX) const;

  void expand_schedule(uint64_t end_time);

  ErrorCode handle_delete_schedule(const nlohmann::json & request_json);
  void delete_schedule(
    const std::vector<std::string> & event_ids,
    const std::vector<std::string> & dependency_ids,
    const std::vector<std::string> & series_ids);

  const data::Schedule & schedule_handler_const() const;
  data::Schedule & schedule_handler();

  // Runtime related functions
  void load_runtime_interface(
    const std::shared_ptr<void> & node,
    const std::string & name,
    const std::string & interface,
    const std::vector<std::string> & supported_tasks);

  void unload_runtime_interface(
    const std::string & name);

  // Estimation related functions
  void load_estimate_interface(
    const std::shared_ptr<void> & node,
    const std::string & name,
    const std::string & interface,
    const std::vector<std::string> & supported_tasks);

  void unload_estimate_interface(
    const std::string & name);

  // Task builder related functions
  void load_builder_interface(
    const std::shared_ptr<void> & node,
    const std::string & name,
    const std::string & interface,
    const std::vector<std::string> & supported_tasks);

  void unload_builder_interface(
    const std::string & name);

  void optimize(uint64_t start_time, uint64_t end_time);

  void spin();
  void stop();

private:
  // A small convenience function for converting a thread ID to a string
  std::string _thread_id();

  mutable std::shared_mutex mtx_;
  data::Schedule schedule_;
  SchedulerOptions options_;
  uint64_t series_expand_time_;

  std::unique_ptr<SchemaValidator> schema_validator_;
  uint64_t last_write_time_;
  std::string last_writer_;

  // Task detail builder
  task::Builder task_builder_;

  // Task estimation interface
  task::Estimator task_estimator_;

  // Task executor interface
  task::Executor task_executor_;
  std::unordered_set<std::string> pushed_action_ids_;

  // Runtime interface
  void _tick();
  void _push_events(uint64_t start_time, uint64_t end_time);
  runtime::SystemTimeExecutor::Action _generate_dag_action(data::DAG & dag);
  runtime::SystemTimeExecutor::Action _generate_event_action(
    const data::Event & event, bool blocking = false);

  // DAG executor and their status
  std::vector<runtime::DAGExecutor> dag_executors_;
  std::vector<std::shared_future<void>> dag_futures_;

  runtime::SystemTimeExecutor ste_;

  uint64_t next_tick_time_;
  double tick_period_;  // ticking period, in seconds
  std::atomic_bool spinning_;
};

namespace exception
{

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
  ScheduleMultipleWriteException(
    const char * msg, Args && ... args)
  : ExceptionTemplate(
      ErrorCode::FAILURE | ErrorCode::RUNTIME | ErrorCode::MULTIPLE_ACCESS,
      msg, std::forward<Args>(args) ...)
  {
    add_prefix("ScheduleMultipleWriteException:\n  ");
  }
};

}  // namespace exception

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__SCHEDULER_HPP_
