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

#include <algorithm>

#include <conflict_resolver/resolver.hpp>

namespace conflict_resolver
{

void Resolver::push_intervals(uint64_t from, uint64_t to)
{
  std::pair<uint64_t, uint64_t> event_pair;
  event_pair.first = from;
  event_pair.second = to;
  events_.push_back(event_pair);
}

void Resolver::clear_intervals()
{
  events_.clear();
}

std::vector<std::pair<uint64_t, uint64_t>> Resolver::resolve()
{
  // sorting based on start time
  std::sort(
    events_.begin(), events_.end(),
    [](const std::pair<uint64_t, uint64_t> & lhs,
    const std::pair<uint64_t, uint64_t> & rhs) -> bool
    {
      return lhs.first < rhs.first;
    });


  // identifying and resolving conflicts in sorted schedules
  for (uint64_t i = 0; i < events_.size(); i++) {
    std::pair<int, int> from = events_[i];
    for (uint64_t j = i + 1; j < events_.size(); j++) {
      std::pair<int, int> with = events_[j];

      // breaking the loop since the vector is sorted based on start time and
      // if the compared schedule is behind next schedule the rest
      // will also be of the same type
      if (with.first > from.second) {break;}

      Identifier identifier;
      uint64_t duration_to_avoid_conflict =
        identifier.conflict_check(from.first, from.second, with.first, with.second);

      if (duration_to_avoid_conflict != 0) {
        with.first = with.first + duration_to_avoid_conflict;
        with.second = with.second + duration_to_avoid_conflict;
        events_[j] = with;
      }
    }
  }

  return events_;
}

}  // namespace conflict_resolver
