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

#ifndef RMF_SCHEDULER__CONFLICT_RESOLVER__IDENTIFIER_HPP_
#define RMF_SCHEDULER__CONFLICT_RESOLVER__IDENTIFIER_HPP_

#include <cstdint>
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>

#include "rmf_scheduler/event.hpp"

using Event = rmf_scheduler::Event;

namespace rmf_scheduler
{
namespace conflict_resolver
{
class Identifier
{
public:
  /// Method to check conflicts in given schedules
  /**
   * Example Conflicts:
   * - schedule a: 10 - 15, schedule b:  5 - 11
   * - schedule a: 10 - 15, schedule b: 14 - 19
   * - schedule a: 10 - 15, schedule b: 12 - 14
   * - schedule a: 10 - 15, schedule b:  8 - 17
   *
   * \param[in] lhs_start_time - Schedule A start time
   * \param[in] lhs_end_time - Schedule A end time
   * \param[in] rhs_start_time - Schedule B start time
   * \param[in] rhs_end_time - Schedule B end time
   * \return[out] Duration of conflict between schedules
   */
  uint64_t conflict_check(
    uint64_t lhs_start_time,
    uint64_t lhs_end_time,
    uint64_t rhs_start_time,
    uint64_t rhs_end_time);

  /// Method to identify conflicts a list of events
  /**
   *
   * \param[in] events - A vector of events
   * \return[out] A list of pairs of ids of conflicting events
   */
  auto identify_conflicts(const std::vector<Event> & events)
  -> std::vector<std::pair<std::string, std::string>>;

  /// Method to identify conflicts a map of events
  /**
   *
   * \param[in] events - An unordered map of events
   * \return[out] A list of pairs of ids of conflicting events
   */
  auto identify_conflicts(const std::unordered_map<std::string, Event> & events)
  -> std::vector<std::pair<std::string, std::string>>;

  /// Method to identify conflicts a list of events
  /**
   *
   * \param[in] events - An unordered map of events
   * \return[out] An unordered map of unordered map of events based on robot names
   */
  auto categorise_by_robots(const std::unordered_map<std::string, Event> & events)
  -> std::unordered_map<std::string, std::unordered_map<std::string, Event>>;

  /// Method to identify conflicts a list of events
  /**
   *
   * \param[in] events - An unordered map of events
   * \return[out] An unordered map of unordered map of events based on location
   */
  auto categorise_by_location(const std::unordered_map<std::string, Event> & events)
  -> std::unordered_map<std::string, std::unordered_map<std::string, Event>>;
};

}  // namespace conflict_resolver
}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__CONFLICT_RESOLVER__IDENTIFIER_HPP_
