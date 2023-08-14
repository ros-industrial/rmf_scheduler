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

#include "rmf_scheduler/conflict_identifier.hpp"
#include "rmf_scheduler/parser.hpp"

namespace rmf_scheduler
{

namespace utils
{

bool simple_conflict_check(
  uint64_t lhs_start_time,
  uint64_t lhs_end_time,
  uint64_t rhs_start_time,
  uint64_t rhs_end_time)
{
  if (rhs_start_time >= lhs_start_time && rhs_start_time < lhs_end_time) {
    return true;
  }

  if (lhs_start_time > rhs_start_time && lhs_start_time < rhs_end_time) {
    return true;
  }

  return false;
}

}  // namespace utils

// Internal functions
namespace internal
{

/// Event with the filtered details
struct EventwCache
{
  EventwCache(
    const Event & _event,
    const std::vector<std::string> & _filters)
  {
    if (!_filters.empty()) {
      // Generate the filtered result for all event qualified
      std::vector<bool> filter_success;
      parser::batch_filter_event_details<std::string>(
        _event.event_details,
        _filters,
        filter_success,
        filtered_details
      );
    }
    filters = _filters;
    event = _event;
  }
  Event event;
  std::vector<std::string> filters;
  std::vector<std::string> filtered_details;
};

using EventwCacheConflictIdentifierFunc =
  std::function<bool (const EventwCache &, const EventwCache &, Conflict &)>;

bool filtered_details_conflict_identifier(
  const EventwCache & event1, const EventwCache & event2, Conflict & conflict)
{
  // Check filter
  if (!event1.filtered_details.empty()) {
    bool overlapped = false;
    for (size_t i = 0; i < event1.filtered_details.size(); i++) {
      if (!event1.filtered_details[i].empty() &&
        event1.filtered_details[i] == event2.filtered_details[i])
      {
        overlapped = true;
        conflict.filter = event1.filters[i];
        conflict.filtered_detail = event1.filtered_details[i];
        break;
      }
    }
    if (!overlapped) {
      return false;
    }
  }

  conflict.first = event1.event.id;
  conflict.second = event2.event.id;
  return true;
}

/// Using multimap to reduce iterating through all the possible combinations
std::vector<Conflict> identify_conflicts_memory_intensive(
  const std::multimap<uint64_t, EventwCache> & event_time_lookup,
  EventwCacheConflictIdentifierFunc filtered_details_conflict_identifier_func)
{
  std::vector<Conflict> conflicts;
  for (auto itr = event_time_lookup.begin(); itr != event_time_lookup.end(); itr++) {
    auto upper_itr = event_time_lookup.upper_bound(itr->first + itr->second.event.duration - 1);

    // Start from the next iterator
    auto overlap_itr = itr;
    overlap_itr++;
    for (; overlap_itr != upper_itr; overlap_itr++) {
      Conflict conflict;
      // Identify conflict
      if (filtered_details_conflict_identifier_func(
          itr->second, overlap_itr->second, conflict))
      {
        // push back the conflict
        conflicts.push_back(conflict);
      }
    }
  }
  return conflicts;
}

/// Iterate through all the possible combinations
std::vector<Conflict> identify_conflicts_greedy(
  const std::vector<EventwCache> & event_time_lookup,
  EventwCacheConflictIdentifierFunc filtered_details_conflict_identifier_func)
{
  std::vector<Conflict> conflicts;
  for (auto itr = event_time_lookup.begin(); itr != event_time_lookup.end(); itr++) {
    // Start from the next iterator
    auto itr2 = itr;
    itr2++;
    for (; itr2 != event_time_lookup.end(); itr2++) {
      // Skip if there is no conflict
      if (!utils::simple_conflict_check(
          itr->event.start_time, itr->event.start_time + itr->event.duration,
          itr2->event.start_time, itr2->event.start_time + itr2->event.duration
      ))
      {
        continue;
      }
      Conflict conflict;
      // Identify conflict
      if (filtered_details_conflict_identifier_func(
          *itr, *itr2, conflict))
      {
        // push back the conflict
        conflicts.push_back(conflict);
      }
    }
  }
  return conflicts;
}

}  // namespace internal

namespace utils
{

std::vector<Conflict> identify_conflicts(
  const std::vector<Event> & events,
  const std::unordered_set<std::string> & allowed_types,
  const std::vector<std::string> & filters,
  const std::string & method)
{
  if (method == "optimal") {
    std::multimap<uint64_t, internal::EventwCache> event_time_lookup;
    for (auto & event : events) {
      // Skip events that doesn't match the allowed type
      if (!allowed_types.empty() &&
        allowed_types.find(event.type) == allowed_types.end())
      {
        continue;
      }

      // Generate the filtered result for the conflict checking algo
      event_time_lookup.emplace(
        event.start_time,
        internal::EventwCache(event, filters));
    }

    return internal::identify_conflicts_memory_intensive(
      event_time_lookup,
      internal::filtered_details_conflict_identifier);
  } else if (method == "greedy") {
    std::vector<internal::EventwCache> event_w_cache;
    for (auto & event : events) {
      // Skip events that doesn't match the allowed type
      if (!allowed_types.empty() &&
        allowed_types.find(event.type) == allowed_types.end())
      {
        continue;
      }

      // Generate the filtered result for the conflict checking algo
      event_w_cache.push_back(
        internal::EventwCache(event, filters));
    }

    return internal::identify_conflicts_greedy(
      event_w_cache,
      internal::filtered_details_conflict_identifier);
  } else {
    throw std::invalid_argument("Conflict identification method unknown");
  }
}

std::vector<std::unordered_map<std::string, std::vector<std::string>>>
categorise_by_filter(
  const std::vector<Event> & events,
  const std::vector<std::string> & filters,
  const std::unordered_set<std::string> & allowed_types)
{
  std::vector<std::unordered_map<std::string, std::vector<std::string>>>
  result(filters.size());
  for (auto & event : events) {
    // Skip events that doesn't match the allowed type
    if (!allowed_types.empty() &&
      allowed_types.find(event.type) == allowed_types.end())
    {
      continue;
    }

    std::vector<std::string> filtered_details;
    std::vector<bool> filter_success;
    parser::batch_filter_event_details<std::string>(
      event.event_details,
      filters,
      filter_success,
      filtered_details
    );
    for (size_t i = 0; i < filters.size(); i++) {
      if (filter_success[i]) {
        result[i][filtered_details[i]].push_back(event.id);
      }
    }
  }
  return result;
}

std::unordered_map<std::string, std::vector<std::string>> categorise_by_type(
  const std::vector<Event> & events,
  const std::unordered_set<std::string> & allowed_types)
{
  std::unordered_map<std::string, std::vector<std::string>> result;
  for (auto & event : events) {
    // Skip events that doesn't match the allowed type
    if (!allowed_types.empty() &&
      allowed_types.find(event.type) == allowed_types.end())
    {
      continue;
    }
    result[event.type].push_back(event.id);
  }
  return result;
}
}  // namespace utils
}  // namespace rmf_scheduler
