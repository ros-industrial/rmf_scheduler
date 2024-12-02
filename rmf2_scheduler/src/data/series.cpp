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

#include <cassert>
#include <set>

#include "croncpp/croncpp.h"

#include "rmf2_scheduler/data/series.hpp"
#include "rmf2_scheduler/data/uuid.hpp"
#include "rmf2_scheduler/utils/utils.hpp"

namespace rmf2_scheduler
{

namespace data
{

Series::Series()
: cron_(std::make_unique<cron::cronexpr>()),
  tz_("UTC"),
  until_(Time::max())
{
}

Series::Series(Series && rhs) = default;

Series & Series::operator=(Series && rhs) = default;

Series::~Series()
{
}

Series::Series(
  const std::string & id,
  const std::string & type,
  const Occurrence & occurrence,
  const std::string & cron,
  const std::string & timezone,
  const Time & until)
: Series(id, type, {occurrence}, cron, timezone, until, {})
{
}

Series::Series(
  const std::string & id,
  const std::string & type,
  const std::vector<Occurrence> & occurrences,
  const std::string & cron,
  const std::string & timezone,
  const Time & until,
  const std::vector<std::string> & exception_ids)
: id_(id), type_(type), tz_(timezone), until_(until)
{
  if (occurrences.empty()) {
    throw std::invalid_argument("Series create failed, no occurrences provided.");
  }

  // Parse the exception first
  for (auto & exception_id : exception_ids) {
    exception_ids_.emplace(exception_id);
  }

  // Create to validate
  // existing occurrence if it has more than 1 occurrence
  try {
    cron_ = std::make_unique<cron::cronexpr>(cron::make_cron(cron));
  } catch (const cron::bad_cronexpr & e) {
    throw std::invalid_argument(
            "Series create failed, "
            "invalid cron [" + cron + "]: " + std::string(e.what())
    );
  }

  for (auto itr : occurrences) {
    // Validate if the timing matches the cron
    if (exception_ids_.find(itr.id) == exception_ids_.end() &&
      !_validate_time(itr.time, cron_))
    {
      throw std::invalid_argument(
              "Series create failed, "
              "invalid time [" + std::string(itr.time.to_localtime()) + "] "
              "doesn't match cron [" + cron + "]."
      );
    }
    occurrence_lookup_.emplace(itr.time.nanoseconds(), itr.id);
  }
}

Series::operator bool() const
{
  if (occurrence_lookup_.empty()) {
    return false;
  }

  return true;
}

const std::string & Series::id() const
{
  return id_;
}

const std::string & Series::type() const
{
  return type_;
}

std::vector<Occurrence> Series::occurrences() const
{
  std::vector<Occurrence> occurrences;
  for (auto & itr : occurrence_lookup_) {
    occurrences.emplace_back(Occurrence{itr.first, itr.second});
  }
  return occurrences;
}

Occurrence Series::get_occurrence(const Time & time) const
{
  auto itr = occurrence_lookup_.find(time.nanoseconds());
  if (itr == occurrence_lookup_.end()) {
    throw std::out_of_range(
            "Series get_occurrence failed, time "
            "[" + std::string(time.to_localtime()) + "] doesn't exist."
    );
  } else {
    return Occurrence{itr->first, itr->second};
  }
}

Occurrence Series::get_occurrence(const std::string & id) const
{
  for (auto & itr : occurrence_lookup_) {
    if (itr.second == id) {
      return Occurrence{itr.first, itr.second};
    }
  }
  throw std::out_of_range(
          "Series get_occurrence failed, ID "
          "[" + id + "] doesn't exist."
  );
}

Occurrence Series::get_last_occurrence() const
{
  if (!*this) {
    throw std::out_of_range(
            "Series get_last_occurrence failed, series empty."
    );
  }
  auto itr = occurrence_lookup_.rbegin();
  return Occurrence{itr->first, itr->second};
}

Occurrence Series::get_first_occurrence() const
{
  if (!*this) {
    throw std::out_of_range(
            "Series get_first_occurrence failed, series empty."
    );
  }
  auto itr = occurrence_lookup_.begin();
  return Occurrence{itr->first, itr->second};
}

const Time & Series::until() const
{
  return until_;
}

std::string Series::cron() const
{
  return cron::to_cronstr(*cron_);
}

std::string Series::tz() const
{
  return tz_;
}

const std::unordered_set<std::string> & Series::exception_ids() const
{
  return exception_ids_;
}

std::vector<std::string> Series::exception_ids_ordered() const
{
  // Order the exceptions based on start time
  std::set<std::string> exception_ids_ordered_set{
    exception_ids_.begin(), exception_ids_.end()
  };
  return {exception_ids_ordered_set.begin(), exception_ids_ordered_set.end()};
}

std::vector<Occurrence> Series::expand_until(const Time & time)
{
  if (!*this) {
    throw std::out_of_range(
            "Series expand_until failed, series empty."
    );
  }

  std::vector<Occurrence> result;
  Time until_time_c = time;
  if (time > until_) {
    until_time_c = until_;
  }

  auto last_itr = occurrence_lookup_.rbegin();
  utils::set_timezone(tz_.c_str());

  time_t last_time_s = Time(last_itr->first).seconds();
  time_t until_time_s = until_time_c.seconds();
  Occurrence ref_occurrence{last_itr->first, last_itr->second};

  last_time_s = cron::cron_next(*cron_, last_time_s);
  while (last_time_s <= until_time_s) {
    Time last_time_c(last_time_s, 0);
    std::string new_id = gen_uuid();
    occurrence_lookup_.emplace(last_time_c.nanoseconds(), new_id);
    Occurrence new_occurrence(last_time_c, new_id);
    result.emplace_back(new_occurrence);

    // Invoke observers: Add occurrence
    for (auto & observer : observers_) {
      observer->on_add_occurrence(
        ref_occurrence,
        new_occurrence
      );
    }
    last_time_s = cron::cron_next(*cron_, last_time_s);
  }

  return result;
}

void Series::update_id(const std::string & new_id)
{
  id_ = new_id;
}

void Series::update_occurrence(
  const Time & time,
  const std::string & new_id)
{
  auto itr = occurrence_lookup_.find(time.nanoseconds());
  if (itr == occurrence_lookup_.end()) {
    throw std::out_of_range(
            "Series update_occurrence failed, time "
            "[" + std::string(time.to_localtime()) + "] doesn't exist."
    );
  }

  std::string old_id = itr->second;
  itr->second = new_id;

  // Update the ID for the exception as well
  auto exception_itr = exception_ids_.find(old_id);
  if (exception_itr != exception_ids_.end()) {
    exception_ids_.erase(exception_itr);
    exception_ids_.emplace(new_id);
  }
}

void Series::update_occurrence(
  const Time & time,
  const Time & new_start_time)
{
  auto itr = occurrence_lookup_.find(time.nanoseconds());
  if (itr == occurrence_lookup_.end()) {
    throw std::out_of_range(
            "Series update_occurrence failed, time "
            "[" + std::string(time.to_localtime()) + "] doesn't exist."
    );
  }

  std::string id = itr->second;

  // Safe delete without invoking delete observer
  _safe_delete(itr->first);

  // Mark this occurrence as an exception
  if (exception_ids_.find(id) == exception_ids_.end()) {
    exception_ids_.emplace(id);
  }

  // Add in new start time
  occurrence_lookup_[new_start_time.nanoseconds()] = id;

  // Invoke observers: Update occurrence
  for (auto & observer : observers_) {
    observer->on_update_occurrence(
      Occurrence{time, id},
      new_start_time);
  }
}


void Series::update_cron_from(
  const Time & time,
  const Time & new_start_time,
  std::string new_cron,
  std::string timezone)
{
  auto itr = occurrence_lookup_.find(time.nanoseconds());

  // Check if there are any occurrence to update
  if (itr == occurrence_lookup_.end()) {
    throw std::out_of_range(
            "Series update_cron_from failed, time "
            "[" + std::string(time.to_localtime()) + "] doesn't exist."
    );
  }

  // Return if no changes
  if (new_cron == cron() && timezone == tz_ && new_start_time == time) {
    return;
  }

  // Check if the occurrence to be updated is already an exception
  if (exception_ids_.find(itr->second) != exception_ids_.end()) {
    throw std::invalid_argument(
            "Series update_cron_from failed, time "
            "[" + std::string(time.to_localtime()) + "] is already an exception."
    );
  }

  std::unique_ptr<cron::cronexpr> temp_cron;
  try {
    temp_cron = std::make_unique<cron::cronexpr>(cron::make_cron(new_cron));
  } catch (const cron::bad_cronexpr & e) {
    throw std::invalid_argument(
            "Series update_cron_from failed, "
            "invalid cron [" + new_cron + "]: " + std::string(e.what())
    );
  }

  // Validate new start time against the new cron
  if (!_validate_time(new_start_time, temp_cron)) {
    throw std::invalid_argument(
            "Series update_cron_from failed, "
            "invalid time [" + std::string(new_start_time.to_localtime()) + "] "
            "doesn't match cron [" + new_cron + "]."
    );
  }

  tz_ = timezone;

  cron_ = std::move(temp_cron);
  time_t cron_time_s = new_start_time.seconds();
  std::vector<Occurrence> new_occurrences {Occurrence{new_start_time, itr->second}};
  for (auto & observer : observers_) {
    observer->on_update_occurrence(
      Occurrence{time, itr->second},
      new_start_time
    );
  }
  itr = occurrence_lookup_.erase(itr);


  // Create new occurrences based on old
  utils::set_timezone(tz_.c_str());
  while (itr != occurrence_lookup_.end()) {
    cron_time_s = cron::cron_next(*cron_, cron_time_s);
    if (exception_ids_.find(itr->second) == exception_ids_.end()) {
      // Push back occurrence that are not mark as an exception
      Time occurrence_time_c(cron_time_s, 0);
      new_occurrences.push_back(Occurrence{occurrence_time_c, itr->second});
      // Invoke observers: Update occurrence
      for (auto & observer : observers_) {
        observer->on_update_occurrence(
          Occurrence{itr->first, itr->second},
          occurrence_time_c);
      }
      itr = occurrence_lookup_.erase(itr);
    } else {
      // Skip the occurrence if it is an exception
      itr++;
    }
  }

  // Add the newly created occurrences back to the series
  for (auto & occurrence : new_occurrences) {
    occurrence_lookup_[occurrence.time.nanoseconds()] = occurrence.id;
  }

  // Set all the occurrences before current time as exception
  auto itr_old_occurrence = occurrence_lookup_.lower_bound(new_start_time.nanoseconds());
  for (auto i = occurrence_lookup_.begin(); i != itr_old_occurrence; i++) {
    if (exception_ids_.find(i->second) == exception_ids_.end()) {
      exception_ids_.emplace(i->second);
    }
  }
}

void Series::update_until(const Time & time)
{
  until_ = time;

  auto itr = occurrence_lookup_.upper_bound(time.nanoseconds());
  while (itr != occurrence_lookup_.end()) {
    rs_time_point_value_t time_ns = itr->first;
    std::string id = itr->second;

    // Remove all occurrences after new until
    itr = occurrence_lookup_.erase(itr);

    // Remove the exceptions as well
    auto exception_itr = exception_ids_.find(id);
    if (exception_itr != exception_ids_.end()) {
      exception_ids_.erase(exception_itr);
    }

    // Invoke observers: Delete occurrence
    for (auto & observer : observers_) {
      observer->on_delete_occurrence(
        Occurrence{time_ns, id}
      );
    }
  }
}

void Series::delete_occurrence(const Time & time)
{
  auto itr = occurrence_lookup_.find(time.nanoseconds());
  if (itr == occurrence_lookup_.end()) {
    throw std::out_of_range(
            "Series delete_occurrence failed, time "
            "[" + std::string(time.to_localtime()) + "] doesn't exist."
    );
  }
  std::string occurrence_id = itr->second;
  _safe_delete(time.nanoseconds());
  // Invoke observers: Delete occurrence
  for (auto & observer : observers_) {
    observer->on_delete_occurrence(
      Occurrence{time, occurrence_id});
  }
}

bool Series::_validate_time(
  const Time & time,
  const std::unique_ptr<cron::cronexpr> & cron) const
{
  utils::set_timezone(tz_.c_str());
  int time_s_to_validate = static_cast<int>(time.seconds());

  // Roll back 1s
  time_t roll_back_time_s = time_s_to_validate - 1;

  int valid_time_s = static_cast<int>(cron::cron_next(*cron, roll_back_time_s));

  return time_s_to_validate == valid_time_s;
}

void Series::_safe_delete(rs_time_point_value_t time_ns)
{
  auto itr = occurrence_lookup_.find(time_ns);
  assert(itr != occurrence_lookup_.end());

  auto last_occurrence = get_last_occurrence();

  // if the occurrence is the last element
  // insert one additional time at the end
  if (time_ns == last_occurrence.time.nanoseconds()) {
    utils::set_timezone(tz_.c_str());
    time_t current_time_s = time_ns / 1e9;
    time_t next_time_s = cron::cron_next(*cron_, current_time_s);
    Time next_time_c(next_time_s, 0);
    // only insert when it is still before the until time
    if (next_time_c <= until_) {
      std::string new_id = gen_uuid();
      occurrence_lookup_.emplace(next_time_c.nanoseconds(), new_id);
      // Invoke observers: Add occurrence
      for (auto & observer : observers_) {
        observer->on_add_occurrence(
          last_occurrence,
          Occurrence{next_time_c, new_id});
      }
    } else {
      until_ = Time(time_ns) - Duration::from_seconds(1);
    }
  }

  std::string id = itr->second;
  occurrence_lookup_.erase(itr);

  // Erase exceptions occurrence if it is considered one
  auto itr_except = exception_ids_.find(id);
  if (itr_except != exception_ids_.end()) {
    exception_ids_.erase(itr_except);
  }
}

}  // namespace data

}  // namespace rmf2_scheduler
