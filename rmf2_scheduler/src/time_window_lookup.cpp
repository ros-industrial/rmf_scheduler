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

#include <algorithm>
#include <cassert>
#include <set>
#include <stdexcept>

#include "rmf2_scheduler/time_window_lookup.hpp"

namespace rmf2_scheduler
{

void TimeWindowLookup::add_entry(
  const Entry & entry
)
{
  if (id_lookup_.find(entry.id) != id_lookup_.end()) {
    throw std::invalid_argument("ID already exist in lookup");
  }

  const data::TimeWindow & time_window = entry.time_window;

  if (time_window.start > time_window.end) {
    throw std::invalid_argument("TimeWindow must be start > end");
  }
  start_lookup_.emplace(time_window.start.nanoseconds(), entry);
  end_lookup_.emplace(time_window.end.nanoseconds(), entry);
  id_lookup_.emplace(entry.id, entry);
}

void TimeWindowLookup::remove_entry(
  const std::string & id
)
{
  // Find the entry
  auto itr = id_lookup_.find(id);
  if (itr == id_lookup_.end()) {
    throw std::invalid_argument("ID doesn't exist in lookup");
  }

  const Entry & entry = itr->second;

  // Remove start lookup
  auto start_range = start_lookup_.equal_range(entry.time_window.start.nanoseconds());
  auto start_lookup_itr = start_range.first;
  for (; start_lookup_itr != start_range.second; start_lookup_itr++) {
    if (start_lookup_itr->second.id == id) {
      start_lookup_.erase(start_lookup_itr);
      break;
    }
  }
  assert(start_lookup_itr != start_range.second);

  // Remove end lookup
  auto end_range = end_lookup_.equal_range(entry.time_window.end.nanoseconds());
  auto end_lookup_itr = end_range.first;
  for (; end_lookup_itr != end_range.second; end_lookup_itr++) {
    if (end_lookup_itr->second.id == id) {
      end_lookup_.erase(end_lookup_itr);
      break;
    }
  }

  id_lookup_.erase(itr);
}

TimeWindowLookup::Entry TimeWindowLookup::get_entry(
  const std::string & id
) const
{
  // Find the entry
  auto itr = id_lookup_.find(id);
  if (itr == id_lookup_.end()) {
    throw std::invalid_argument("ID doesn't exist in lookup");
  }

  return itr->second;
}

std::vector<TimeWindowLookup::Entry> TimeWindowLookup::lookup(
  const data::TimeWindow & time_window,
  bool soft_lower_bound,
  bool soft_upper_bound
) const
{
  if (time_window.start > time_window.end) {
    return {};
  }

  std::vector<Entry> result;
  result.reserve(id_lookup_.size());

  if (!soft_lower_bound) {
    // Retieve entry with start time within the time window
    auto start_lower_itr = start_lookup_.lower_bound(time_window.start.nanoseconds());
    auto start_upper_itr = start_lookup_.upper_bound(time_window.end.nanoseconds());

    for (auto itr = start_lower_itr; itr != start_upper_itr; itr++) {
      auto id = itr->second.id;

      // Remove entries violates hard upper bound constraint
      if (!soft_upper_bound && id_lookup_.at(id).time_window.end > time_window.end) {
        continue;
      }

      result.push_back(itr->second);
    }
    return result;
  }

  // Retieve entry with end time within the time window
  auto end_lower_itr = end_lookup_.lower_bound(time_window.start.nanoseconds());
  auto end_upper_itr = end_lookup_.upper_bound(time_window.end.nanoseconds());

  for (auto itr = end_lower_itr; itr != end_upper_itr; itr++) {
    result.push_back(itr->second);
  }

  if (!soft_upper_bound) {
    return result;
  }

  // Retieve entry with start time within the time window
  auto start_upper_itr = start_lookup_.upper_bound(time_window.end.nanoseconds());

  // Add in missed entries
  for (auto itr = start_lookup_.begin(); itr != start_upper_itr; itr++) {
    auto id = itr->second.id;
    if (id_lookup_.at(id).time_window.end > time_window.end) {
      result.push_back(itr->second);
    }
  }

  return result;
}

bool TimeWindowLookup::empty() const
{
  return id_lookup_.empty();
}

data::TimeWindow TimeWindowLookup::get_time_window(
  bool soft_upper_bound
) const
{
  if (empty()) {
    throw std::out_of_range("No entries in TimeWindowLookup, cannot retrieve time_window!");
  }

  data::TimeWindow time_window;
  time_window.start = data::Time(start_lookup_.begin()->first);

  if (soft_upper_bound) {
    time_window.end = data::Time(start_lookup_.rbegin()->first);
  } else {
    time_window.end = data::Time(end_lookup_.rbegin()->first);
  }

  return time_window;
}

}  // namespace rmf2_scheduler
