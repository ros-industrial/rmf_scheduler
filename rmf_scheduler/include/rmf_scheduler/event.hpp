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

#ifndef RMF_SCHEDULER__EVENT_HPP_
#define RMF_SCHEDULER__EVENT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace rmf_scheduler
{

/// Basic information about the Event
struct Event
{
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
};


}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__EVENT_HPP_
