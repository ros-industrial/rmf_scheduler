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

#ifndef RMF_SCHEDULER__CONFLICT_IDENTIFIER_HPP_
#define RMF_SCHEDULER__CONFLICT_IDENTIFIER_HPP_

#include <cstdint>
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "rmf_scheduler/event.hpp"

namespace rmf_scheduler
{

struct Conflict
{
  std::string first;
  std::string second;

  // Reason for Conflict
  std::string filter;
  std::string filtered_detail;
};


namespace utils
{

bool simple_conflict_check(
  uint64_t lhs_start_time,
  uint64_t lhs_end_time,
  uint64_t rhs_start_time,
  uint64_t rhs_end_time);

/// Method to identify conflicts within a list of events
/**
 * Only event with types included in the allowed_types are used for conflict checking.
 * If allowed_types is empty, all events are used for conflict checking
 *
 * If filters are empty, 2 events are considered in conflict if their time overlaps.
 *
 * If filters are not empty, 2 overlapping events are only considered to be in conflict
 * when one of their filtered result of the event details are the same.
 *
 * Currently, the methods available are
 * - memory_intensive (default)
 * - greedy
 *
 * \param[in] events events to detect conflicts
 * \param[in] allowed_types allowed event type.
 * \param[in] filters A list of filters to determine conflicts.
 * \param[in] method method used for identifying conflicts.
 * \return[out] A list of pairs of ids of conflicting events
 */
std::vector<Conflict> identify_conflicts(
  const std::vector<Event> & events,
  const std::unordered_set<std::string> & allowed_types = {},
  const std::vector<std::string> & filters = {},
  const std::string & method = "optimal");

/// Method to categorise events based on filter
/**
 * Only event with types included in the allowed_types are used for categorization.
 * If allowed_types is empty, all events are used for categorization
 *
 * \param[in] events events to detect conflicts
 * \param[in] filters A list of filters to perform categorisation
 * \param[in] allowed_types allowed event type.
 * \return[out] id based on the filtered result
 */
std::vector<std::unordered_map<std::string, std::vector<std::string>>>
categorise_by_filter(
  const std::vector<Event> & events,
  const std::vector<std::string> & filters,
  const std::unordered_set<std::string> & allowed_types = {});

/// Method to categorise events based on type
/**
 * \param[in] allowed_types allowed event type.
 * \return[out] id based on the filtered result
 */
std::unordered_map<std::string, std::vector<std::string>> categorise_by_type(
  const std::vector<Event> & events,
  const std::unordered_set<std::string> & allowed_types = {});

}  // namespace utils

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__CONFLICT_IDENTIFIER_HPP_
