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

#include "croncpp/croncpp.h"

#include "rmf_scheduler/data/series.hpp"
#include "rmf_scheduler/utils/system_time_utils.hpp"
#include "rmf_scheduler/utils/uuid.hpp"

namespace rmf_scheduler
{

namespace data
{

Series::Series()
: cron_(std::make_unique<cron::cronexpr>()),
  tz_("UTC"),
  until_(0),
  id_prefix_("")
{
}

Series::Series(const Series & rhs)
{
  cron_ = std::make_unique<cron::cronexpr>(*rhs.cron_);
  tz_ = rhs.tz_;
  until_ = rhs.until_;
  occurrence_ids_ = rhs.occurrence_ids_;
  id_prefix_ = rhs.id_prefix_;
  exception_ids_ = rhs.exception_ids_;
  observers_ = rhs.observers_;
}

Series & Series::operator=(const Series & rhs)
{
  cron_ = std::make_unique<cron::cronexpr>(*rhs.cron_);
  tz_ = rhs.tz_;
  until_ = rhs.until_;
  occurrence_ids_ = rhs.occurrence_ids_;
  id_prefix_ = rhs.id_prefix_;
  exception_ids_ = rhs.exception_ids_;
  observers_ = rhs.observers_;
  return *this;
}

Series::Series(Series && rhs)
{
  cron_ = std::move(rhs.cron_);
  tz_ = rhs.tz_;
  until_ = rhs.until_;
  occurrence_ids_ = rhs.occurrence_ids_;
  id_prefix_ = rhs.id_prefix_;
  exception_ids_ = rhs.exception_ids_;
  observers_ = rhs.observers_;
}

Series & Series::operator=(Series && rhs)
{
  cron_ = std::move(rhs.cron_);
  tz_ = rhs.tz_;
  until_ = rhs.until_;
  occurrence_ids_ = rhs.occurrence_ids_;
  id_prefix_ = rhs.id_prefix_;
  exception_ids_ = rhs.exception_ids_;
  observers_ = rhs.observers_;
  return *this;
}

Series::~Series()
{
}

Series::Series(const Series::Description & description)
{
  if (description.occurrences.empty()) {
    throw exception::SeriesEmptyException("Cannot create series from empty description");
  }

  // Parse the exception first
  for (auto & exception_id : description.exception_ids) {
    exception_ids_.emplace(exception_id);
  }

  // Create to validate
  // existing occurrence if it has more than 1 occurrence
  try {
    cron_ = std::make_unique<cron::cronexpr>(cron::make_cron(description.cron));
  } catch (const cron::bad_cronexpr & e) {
    throw exception::SeriesInvalidCronException(
            "Cannot create series. "
            "Invalid cron: %s", e.what());
  }

  tz_ = description.timezone;

  for (auto itr : description.occurrences) {
    // Validate if the timing matches the cron
    if (exception_ids_.find(itr.id) == exception_ids_.end() &&
      !_validate_time(itr.time, cron_))
    {
      throw exception::SeriesInvalidCronException(
              "Cannot create series. "
              "Invalid time \n\t%s for event [%s] doesn't match cron '%s'.",
              utils::to_localtime(itr.time),
              itr.id.c_str(), description.cron.c_str());
    }
    occurrence_ids_.emplace(itr.time, itr.id);
  }

  until_ = description.until;
  id_prefix_ = description.id_prefix;
}

Series::Series(
  const std::string & id,
  uint64_t time,
  const std::string & cron,
  const std::string & timezone,
  uint64_t until,
  const std::string & id_prefix)
: until_(until),
  id_prefix_(id_prefix)
{
  // Create cron to validate
  // existing occurrence if it has more than 1 occurrence
  try {
    cron_ = std::make_unique<cron::cronexpr>(cron::make_cron(cron));
  } catch (const cron::bad_cronexpr & e) {
    throw exception::SeriesInvalidCronException(
            "Cannot create series. "
            "Invalid cron: %s", e.what());
  }
  tz_ = timezone;
  if (!_validate_time(time, cron_)) {
    throw exception::SeriesInvalidCronException(
            "Cannot create series. "
            "Invalid time \n\t%s for event [%s] doesn't match cron '%s'",
            utils::to_localtime(time),
            id.c_str(), cron.c_str());
  }
  occurrence_ids_.emplace(time, id);
}

Series::operator bool() const
{
  if (!cron_) {
    return false;
  }

  if (occurrence_ids_.empty()) {
    return false;
  }

  return true;
}

Series::Description Series::description() const
{
  Description description;
  description.occurrences = occurrences();
  description.cron = cron();
  description.timezone = tz_;
  description.until = until_;
  for (auto & itr : exception_ids_) {
    description.exception_ids.push_back(itr);
  }
  description.id_prefix = id_prefix_;
  return description;
}

std::vector<Series::Occurrence> Series::occurrences() const
{
  std::vector<Occurrence> occurrences;
  for (auto & itr : occurrence_ids_) {
    occurrences.emplace_back(Occurrence{itr.first, itr.second});
  }
  return occurrences;
}

std::string Series::get_occurrence_id(uint64_t time) const
{
  auto itr = occurrence_ids_.find(time);
  if (itr == occurrence_ids_.end()) {
    return "";
  } else {
    return itr->second;
  }
}

uint64_t Series::get_occurrence_time(const std::string & id) const
{
  for (auto & itr : occurrence_ids_) {
    if (itr.second == id) {
      return itr.first;
    }
  }
  throw exception::SeriesIDException(
          "get_occurrence_time",
          "Occurrence ID [%s] not found.",
          id.c_str());
}

Series::Occurrence Series::get_last_occurrence() const
{
  if (!*this) {
    throw exception::SeriesEmptyException(
            "get_last_occurrence: ",
            "No last occurrence found, Series empty.");
  }
  auto itr = occurrence_ids_.rbegin();
  return Series::Occurrence{itr->first, itr->second};
}

Series::Occurrence Series::get_first_occurrence() const
{
  if (!*this) {
    throw exception::SeriesEmptyException(
            "get_first_occurrence: ",
            "No last occurrence found, Series empty.");
  }
  auto itr = occurrence_ids_.begin();
  return Series::Occurrence{itr->first, itr->second};
}

uint64_t Series::get_until() const
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

std::vector<Series::Occurrence> Series::expand_until(uint64_t time)
{
  std::vector<Occurrence> result;
  if (!*this) {
    throw exception::SeriesEmptyException(
            "expand_to: ",
            "Cannot expand series, Series empty.");
  }

  if (time > until_) {
    time = until_;
  }

  auto last_itr = occurrence_ids_.rbegin();
  utils::set_timezone(tz_.c_str());

  time_t last_time = last_itr->first / 1e9;
  time_t until_time = time / 1e9;
  Occurrence ref_occurrence{last_itr->first, last_itr->second};

  last_time = cron::cron_next(*cron_, last_time);
  while (last_time <= until_time) {
    uint64_t ns_time = static_cast<uint64_t>(last_time * 1e9);
    std::string new_id = id_prefix_ + utils::gen_uuid();
    occurrence_ids_.emplace(ns_time, new_id);
    result.emplace_back(Occurrence{ns_time, new_id});
    // Invoke observers: Add occurrence
    for (auto & observer : observers_) {
      observer->on_add_occurrence(
        ref_occurrence,
        Occurrence{ns_time, new_id});
    }
    last_time = cron::cron_next(*cron_, last_time);
  }

  return result;
}

void Series::update_occurrence_id(
  uint64_t time,
  const std::string & new_id)
{
  auto itr = occurrence_ids_.find(time);
  if (itr == occurrence_ids_.end()) {
    throw exception::SeriesInvalidCronException(
            "update_occurrence_id: "
            "Occurrence at time \n\t%s doesn't exist",
            utils::to_localtime(time));
  }

  itr->second = new_id;
}

void Series::update_cron_from(
  uint64_t time,
  uint64_t new_start_time,
  std::string new_cron,
  std::string timezone)
{
  if (new_cron == cron()) {
    return;
  }

  auto itr = occurrence_ids_.lower_bound(time);

  // Check if there are any occurrence to update
  if (itr == occurrence_ids_.end()) {
    throw exception::SeriesInvalidCronException(
            "update_cron_from: "
            "Occurrence at time \n\t%s doesn't exist",
            utils::to_localtime(time));
  }

  // Check if the occurrence to be updated is already an exception
  if (exception_ids_.find(itr->second) != exception_ids_.end()) {
    throw exception::SeriesInvalidCronException(
            "update_cron_from: "
            "Occurrence at time \n\t%s is already an exception",
            utils::to_localtime(time));
  }

  // Validate new start time against the new cron
  auto temp_cron = std::make_unique<cron::cronexpr>(cron::make_cron(new_cron));
  if (!_validate_time(new_start_time, temp_cron)) {
    throw exception::SeriesInvalidCronException(
            "update_cron_from:"
            "Invalid new start time \n\t%s for event [%s] doesn't match cron '%s'",
            utils::to_localtime(new_start_time),
            itr->second.c_str(), new_cron.c_str());
  }

  tz_ = timezone;

  cron_ = std::move(temp_cron);
  time_t cron_time_s = new_start_time / 1e9;
  std::vector<Occurrence> new_occurrences {{new_start_time, itr->second}};
  for (auto & observer : observers_) {
    observer->on_update_occurrence(
      Occurrence{time, itr->second},
      new_start_time);
  }
  for (auto & observer : observers_) {
    observer->on_update_cron(
      Occurrence{time, itr->second},
      new_start_time);
  }
  itr = occurrence_ids_.erase(itr);


  // Create new occurrences based on old
  utils::set_timezone(tz_.c_str());
  while (itr != occurrence_ids_.end()) {
    cron_time_s = cron::cron_next(*cron_, cron_time_s);
    if (exception_ids_.find(itr->second) == exception_ids_.end()) {
      // Push back occurrence that are not mark as an exception
      uint64_t occurrence_time = static_cast<uint64_t>(cron_time_s) * 1e9;
      new_occurrences.push_back(Occurrence{occurrence_time, itr->second});
      // Invoke observers: Update occurrence
      for (auto & observer : observers_) {
        observer->on_update_occurrence(
          Occurrence{itr->first, itr->second},
          occurrence_time);
      }
      for (auto & observer : observers_) {
        observer->on_update_cron(
          Occurrence{itr->first, itr->second},
          occurrence_time);
      }
      itr = occurrence_ids_.erase(itr);
    } else {
      // Skip the occurrence if it is an exception
      itr++;
    }
  }

  // Add the newly created occurrences back to the series
  for (auto & occurrence : new_occurrences) {
    occurrence_ids_[occurrence.time] = occurrence.id;
  }

  // Set all the occurrences before current time as exception
  auto itr_old_occurrence = occurrence_ids_.lower_bound(new_start_time);
  for (auto i = occurrence_ids_.begin(); i != itr_old_occurrence; i++) {
    if (exception_ids_.find(i->second) == exception_ids_.end()) {
      exception_ids_.emplace(i->second);
    }
  }
}


void Series::update_occurrence_time(
  uint64_t time,
  uint64_t new_start_time)
{
  auto itr = occurrence_ids_.find(time);
  if (itr == occurrence_ids_.end()) {
    throw exception::SeriesInvalidCronException(
            "update_occurrence_time: "
            "Occurrence at time \n\t%s doesn't exist",
            utils::to_localtime(time));
  }

  std::string id = itr->second;

  // Safe delete without invoking delete observer
  _safe_delete(itr->first);

  // Mark this occurrence as an exception
  if (exception_ids_.find(id) == exception_ids_.end()) {
    exception_ids_.emplace(id);
  }

  // Add in new start time
  occurrence_ids_[new_start_time] = id;

  // Invoke observers: Update occurrence
  for (auto & observer : observers_) {
    observer->on_update_occurrence(
      Occurrence{time, id},
      new_start_time);
  }
}

void Series::set_id_prefix(
  const std::string & id_prefix)
{
  id_prefix_ = id_prefix;
}

void Series::set_until(
  uint64_t time)
{
  until_ = time;
  auto itr = occurrence_ids_.lower_bound(time);
  // Invoke observers: Delete occurrence
  for (auto & observer : observers_) {
    observer->on_delete_occurrence(
      Occurrence{itr->first, itr->second});
  }
  occurrence_ids_.erase(itr, occurrence_ids_.end());
}

void Series::delete_occurrence(uint64_t time)
{
  std::string occurrence_id = occurrence_ids_.find(time)->second;
  _safe_delete(time);
  // Invoke observers: Delete occurrence
  for (auto & observer : observers_) {
    observer->on_delete_occurrence(
      Occurrence{time, occurrence_id});
  }
}

bool Series::_validate_time(
  uint64_t time,
  const std::unique_ptr<cron::cronexpr> & cron) const
{
  utils::set_timezone(tz_.c_str());
  uint64_t time_s_to_validate = time / 1e9;

  // Roll back 1s
  time_t roll_back_time = time_s_to_validate - 1;

  time_t valid_time = cron::cron_next(*cron, roll_back_time);

  return time_s_to_validate == static_cast<uint64_t>(valid_time);
}

void Series::_safe_delete(uint64_t time)
{
  auto itr = occurrence_ids_.find(time);
  if (itr == occurrence_ids_.end()) {
    throw exception::SeriesInvalidCronException(
            "update_occurrence_id: "
            "Occurrence at time \n\t%s doesn't exist",
            utils::to_localtime(time));
  }

  auto last_occurrence = get_last_occurrence();

  // if the occurrence is the last element
  // insert one additional time at the end
  if (time == last_occurrence.time) {
    utils::set_timezone(tz_.c_str());
    time_t current_time_s = time / 1e9;
    time_t next_time_s = cron::cron_next(*cron_, current_time_s);
    uint64_t next_time =
      static_cast<uint64_t>(next_time_s) * 1e9;
    // only insert when it is still before the until time
    if (next_time <= until_) {
      std::string new_id = id_prefix_ + utils::gen_uuid();
      occurrence_ids_.emplace(next_time, new_id);
      // Invoke observers: Add occurrence
      for (auto & observer : observers_) {
        observer->on_add_occurrence(
          last_occurrence,
          Occurrence{next_time, new_id});
      }
    } else {
      until_ = time - 1;
    }
  }

  std::string id = itr->second;
  occurrence_ids_.erase(itr);

  // Erase exceptions occurrence if it is considered one
  auto itr_except = exception_ids_.find(id);
  if (itr_except != exception_ids_.end()) {
    exception_ids_.erase(itr_except);
  }
}

}  // namespace data

}  // namespace rmf_scheduler
