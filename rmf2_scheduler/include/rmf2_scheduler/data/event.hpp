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

#ifndef RMF2_SCHEDULER__DATA__EVENT_HPP_
#define RMF2_SCHEDULER__DATA__EVENT_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rmf2_scheduler/data/time.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace data
{

/// Basic information about the Event
struct Event
{
  RS_SMART_PTR_DEFINITIONS(Event)

  /// Empty Constructor
  Event() = default;

  /// Minimal Constructor
  /**
   * Constructor with the minimal event information.
   */
  explicit Event(
    const std::string & _id,
    const std::string & _type,
    const Time & _start_time)
  : id(_id), type(_type), start_time(_start_time)
  {
  }

  /// Full Constructor
  /**
   * Constructor with the minimal event information.
   */
  explicit Event(
    const std::string & _id,
    const std::string & _type,
    const std::string & _description,
    const Time & _start_time,
    const Duration & _duration,
    const std::string & _series_id,
    const std::string & _process_id)
  : id(_id),
    type(_type),
    description(_description),
    start_time(_start_time),
    duration(_duration),
    series_id(_series_id),
    process_id(_process_id)
  {
  }

  virtual ~Event() = default;

  /// Event ID
  /*
   * This ID is unique for each event
   */
  std::string id;

  /// Event type
  std::string type;

  /// Event description
  std::optional<std::string> description;

  /// Start time
  Time start_time;

  /// duration
  std::optional<Duration> duration;

  /// Series ID
  std::optional<std::string> series_id;

  /// Process ID
  std::optional<std::string> process_id;

  inline bool operator==(const Event & rhs) const
  {
    if (this->id != rhs.id) {return false;}
    if (this->type != rhs.type) {return false;}
    if (this->description != rhs.description) {return false;}
    if (this->start_time != rhs.start_time) {return false;}
    if (this->duration != rhs.duration) {return false;}
    if (this->series_id != rhs.series_id) {return false;}
    if (this->process_id != rhs.process_id) {return false;}
    return true;
  }

  inline bool operator!=(const Event & rhs) const
  {
    return !(*this == rhs);
  }
};

}  // namespace data

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__DATA__EVENT_HPP_
