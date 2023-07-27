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

#include "rmf_scheduler/conflict_resolver/identifier.hpp"

#include <cmath>
#include <iostream>

namespace rmf_scheduler
{
namespace conflict_resolver
{

uint64_t Identifier::conflict_check(
  uint64_t lhs_start_time,
  uint64_t lhs_end_time,
  uint64_t rhs_start_time,
  uint64_t rhs_end_time)
{
  uint64_t duration_to_avoid_conflict = 0;

  if ((rhs_start_time > lhs_start_time && rhs_start_time < lhs_end_time) ||
    (rhs_end_time > lhs_start_time && rhs_end_time < lhs_end_time) ||
    (rhs_start_time < lhs_start_time && rhs_end_time > lhs_end_time))
  {
    // calculating duration for avoiding conflict
    duration_to_avoid_conflict = abs(rhs_start_time - lhs_end_time) + 1;
  }
  return duration_to_avoid_conflict;
}

auto Identifier::identify_conflicts(const std::vector<rmf_scheduler::Event> & events)
-> std::vector<std::pair<std::string, std::string>>
{
  std::vector<std::pair<std::string, std::string>> conflicts;
  for (int i = 0; i < events.size() - 1; i++) {
    rmf_scheduler::Event lhs = events[i];
    for (int j = i + 1; j < events.size(); j++) {
      rmf_scheduler::Event rhs = events[j];
      if (conflict_check(
          lhs.start_time, lhs.duration,
          rhs.start_time, rhs.duration) != 0)
      {
        conflicts.push_back({lhs.id, rhs.id});
      }
    }
  }
  return conflicts;
}

auto Identifier::identify_conflicts(
  const std::unordered_map<std::string,
  rmf_scheduler::Event> & events)
-> std::vector<std::pair<std::string, std::string>>
{
  std::vector<std::pair<std::string, std::string>> conflicts;
  for (auto i = events.begin(); i != std::next(events.begin(), events.size() - 1); i++) {
    auto & lhs = i->second;
    for (auto j = std::next(i, 1); j != events.end(); j++) {
      auto & rhs = j->second;
      if (conflict_check(
          lhs.start_time, lhs.duration,
          rhs.start_time, rhs.duration) != 0)
      {
        conflicts.push_back({lhs.id, rhs.id});
      }
    }
  }
  return conflicts;
}

auto Identifier::categorise_by_robots(const std::unordered_map<std::string, Event> & events)
-> std::unordered_map<std::string, std::unordered_map<std::string, Event>>
{
  std::unordered_map<std::string, std::unordered_map<std::string, Event>> categorised_events;
  for (auto event : events) {
    if (event.second.is_flight_schedule) {
      // this event is not part of any robot's tasks
      continue;
    }
    categorised_events[event.second.robot][event.second.id] = event.second;
  }
  return categorised_events;
}

auto Identifier::categorise_by_location(const std::unordered_map<std::string, Event> & events)
-> std::unordered_map<std::string, std::unordered_map<std::string, Event>>
{
  std::unordered_map<std::string, std::unordered_map<std::string, Event>> categorised_events;
  for (auto event : events) {
    categorised_events[event.second.location][event.second.id] = event.second;
  }
  return categorised_events;
}

}  // namespace conflict_resolver
}  // namespace rmf_scheduler
