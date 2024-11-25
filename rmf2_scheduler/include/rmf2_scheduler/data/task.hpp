// Copyright 2024 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__DATA__TASK_HPP_
#define RMF2_SCHEDULER__DATA__TASK_HPP_

#include <memory>
#include <string>
#include <optional>

#include "nlohmann/json.hpp"
#include "rmf2_scheduler/data/event.hpp"

namespace rmf2_scheduler
{

namespace data
{

/// Additional information for a Task
struct Task : public Event
{
  using Ptr = std::shared_ptr<Task>;
  using ConstPtr = std::shared_ptr<const Task>;
  using UPtr = std::unique_ptr<Task>;
  using ConstUPtr = std::unique_ptr<const Task>;

  /// Empty Constructor
  Task() = default;

  /// Minimal Constructor
  /**
   * Constructor with the minimal task information.
   */
  explicit Task(
    const std::string & _id,
    const std::string & _type,
    const Time & _start_time,
    const std::string & _status)
  : Event(_id, _type, _start_time), status(_status)
  {
  }

  /// Minimal Constructor
  /**
   * Constructor with the minimal task information.
   */
  explicit Task(
    const Event & event,
    const std::string & _status)
  : Event(event), status(_status)
  {
  }

  /// Full Constructor
  /**
   * Constructor with the full tasks information.
   */
  explicit Task(
    const std::string & _id,
    const std::string & _type,
    const std::string & _description,
    const Time & _start_time,
    const Duration & _duration,
    const std::string & _series_id,
    const std::string & _process_id,
    const std::string & _resource_id,
    const Time & _deadline,
    const std::string & _status,
    const Time & _planned_start_time,
    const Duration & _planned_duration,
    const Duration & _estimated_duration,
    const Time & _actual_start_time,
    const Duration & _actual_duration,
    const nlohmann::json & _task_details
  )
  : Event(
      _id,
      _type,
      _description,
      _start_time,
      _duration,
      _series_id,
      _process_id
  ),
    resource_id(_resource_id),
    deadline(_deadline),
    status(_status),
    planned_start_time(_planned_start_time),
    planned_duration(_planned_duration),
    estimated_duration(_estimated_duration),
    actual_start_time(_actual_start_time),
    actual_duration(_actual_duration),
    task_details(_task_details)
  {
  }

  /// Full Constructor
  /**
   * Constructor with the full tasks information.
   */
  explicit Task(
    const Event & event,
    const std::string & _resource_id,
    const Time & _deadline,
    const std::string & _status,
    const Time & _planned_start_time,
    const Duration & _planned_duration,
    const Duration & _estimated_duration,
    const Time & _actual_start_time,
    const Duration & _actual_duration,
    const nlohmann::json & _task_details
  )
  : Event(event),
    resource_id(_resource_id),
    deadline(_deadline),
    status(_status),
    planned_start_time(_planned_start_time),
    planned_duration(_planned_duration),
    estimated_duration(_estimated_duration),
    actual_start_time(_actual_start_time),
    actual_duration(_actual_duration),
    task_details(_task_details)
  {
  }

  /// Resource ID
  std::optional<std::string> resource_id;

  /// Deadline
  std::optional<Time> deadline;

  /// Task status
  std::string status;

  /// Planned start time
  std::optional<Time> planned_start_time;

  /// Planned duration
  std::optional<Duration> planned_duration;

  /// Estimated Duration
  std::optional<Duration> estimated_duration;

  /// Actual start time
  std::optional<Time> actual_start_time;

  /// Actual duration time
  std::optional<Duration> actual_duration;

  // TODO(Briancbn): remove dependencies on nlohmann/json.
  /// Task Details
  nlohmann::json task_details;

  inline bool operator==(const Task & rhs) const
  {
    if (Event::operator!=(rhs)) {return false;}
    if (this->resource_id != rhs.resource_id) {return false;}
    if (this->deadline != rhs.deadline) {return false;}
    if (this->status != rhs.status) {return false;}
    if (this->planned_start_time != rhs.planned_start_time) {return false;}
    if (this->planned_duration != rhs.planned_duration) {return false;}
    if (this->estimated_duration != rhs.estimated_duration) {return false;}
    if (this->actual_start_time != rhs.actual_start_time) {return false;}
    if (this->actual_duration != rhs.actual_duration) {return false;}
    if (this->task_details != rhs.task_details) {return false;}
    return true;
  }

  inline bool operator!=(const Task & rhs) const
  {
    return !(*this == rhs);
  }
};

}  // namespace data

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__DATA__TASK_HPP_
