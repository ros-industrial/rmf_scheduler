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

#ifndef RMF_SCHEDULER__DATA__EVENT_HPP_
#define RMF_SCHEDULER__DATA__EVENT_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rmf_scheduler
{

namespace data
{

/// Basic information about the Event
struct Event
{
  explicit Event(
    std::string _description,
    std::string _type,
    uint64_t _start_time,
    uint64_t _duration,
    std::string _id,
    std::string _series_id,
    std::string _dag_id,
    std::string _event_details);

  Event() = default;

  /// Event description
  std::string description;

  /// Event type
  std::string type;

  /// Start time
  uint64_t start_time;

  /// Duration
  uint64_t duration;

  /// Event ID
  /*
   * This ID is unique for each event
   */
  std::string id;

  /// Series that the event belongs to
  std::string series_id;

  /// DAG that the event belongs to
  std::string dag_id;

  /// Event details
  std::string event_details;

  /// Task details
  std::string task_details;
};

}  // namespace data

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__DATA__EVENT_HPP_
