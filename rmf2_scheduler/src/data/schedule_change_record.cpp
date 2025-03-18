// Copyright 2025 ROS Industrial Consortium Asia Pacific
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
#include <map>
#include <stdexcept>
#include <unordered_set>
#include "rmf2_scheduler/data/schedule_change_record.hpp"

namespace rmf2_scheduler
{

namespace data
{

// data type
const char record_data_type::EVENT[] = "event";
const char record_data_type::TASK[] = "task";
const char record_data_type::PROCESS[] = "process";
const char record_data_type::SERIES[] = "series";

// action type
const char record_action_type::ADD[] = "add";
const char record_action_type::UPDATE[] = "update";
const char record_action_type::DELETE[] = "delete";

namespace
{

void append_changes(
  std::vector<ChangeAction> & old_changes,
  const std::vector<ChangeAction> & new_changes
)
{
  old_changes.reserve(old_changes.size() + new_changes.size());
  for (auto & changes : new_changes) {
    old_changes.push_back(changes);
  }
}

void assert_valid_change_action(
  const std::string & action,
  const std::unordered_set<std::string> & valid_change_action = {
        record_action_type::ADD,
        record_action_type::UPDATE,
        record_action_type::DELETE,
      }
)
{
  if (valid_change_action.find(action) == valid_change_action.end()) {
    throw std::runtime_error(
            "Invalid ChangeAction action type: [" + action + "]."
    );
  }
}

void squash_changes_once(
  std::map<std::string, std::string> & last_change_id_lookup,
  const ChangeAction & change
)
{
  assert_valid_change_action(change.action);

  // Check id the change ID exist
  auto itr = last_change_id_lookup.find(change.id);

  // Add the changes if no change for the ID exists
  if (itr == last_change_id_lookup.end()) {
    last_change_id_lookup[change.id] = change.action;
    return;
  }

  // Squash the changes if ID exist
  if (itr->second == record_action_type::ADD) {
    // ADD + ADD, invalid
    // ADD + UPDATE = ADD
    // ADD + DELETE = no action
    if (change.action == record_action_type::ADD) {
      throw std::runtime_error("Invalid record to squash, ADD + ADD.");
    } else if (change.action == record_action_type::UPDATE) {
      return;
    } else if (change.action == record_action_type::DELETE) {
      last_change_id_lookup.erase(itr);
      return;
    } else {assert(false);}
  } else if (itr->second == record_action_type::UPDATE) {
    // UPDATE + ADD, invalid
    // UPDATE + UPDATE = UPDATE
    // UPDATE + DELETE = DELETE
    if (change.action == record_action_type::ADD) {
      throw std::runtime_error("Invalid record to squash, UPDATE + UPDATE.");
    } else if (change.action == record_action_type::UPDATE) {
      return;
    } else if (change.action == record_action_type::DELETE) {
      itr->second = record_action_type::DELETE;
      return;
    } else {assert(false);}
  } else if (itr->second == record_action_type::DELETE) {
    // DELETE + ADD = UPDATE
    // DELETE + UPDATE, invalid
    // DELETE + DELETE, invalid
    if (change.action == record_action_type::ADD) {
      itr->second = record_action_type::UPDATE;
      return;
    } else if (change.action == record_action_type::UPDATE) {
      throw std::runtime_error("Invalid record to squash, DELETE + UPDATE.");
    } else if (change.action == record_action_type::DELETE) {
      throw std::runtime_error("Invalid record to squash, DELETE + DELETE.");
    } else {assert(false);}
  }
}

void squash_record_by_data_type(
  const std::string & data_type,
  const std::vector<ScheduleChangeRecord> & records,
  ScheduleChangeRecord & result
)
{
  std::map<std::string, std::string> last_changes;
  for (const auto & record : records) {
    for (const auto & change : record.get(data_type)) {
      squash_changes_once(last_changes, change);
    }
  }

  std::vector<ChangeAction> actions;
  actions.reserve(last_changes.size());
  for (const auto & change_itr : last_changes) {
    actions.push_back(
      ChangeAction{
            change_itr.first,
            change_itr.second
          }
    );
  }

  result.add(data_type, actions);
}

}  // namespace

void ScheduleChangeRecord::add(
  const std::string & type,
  const std::vector<ChangeAction> & changes
)
{
  if (type == record_data_type::EVENT || type == record_data_type::TASK) {
    append_changes(event_changes_, changes);
  } else if (type == record_data_type::PROCESS) {
    append_changes(process_changes_, changes);
  } else if (type == record_data_type::SERIES) {
    append_changes(series_changes_, changes);
  } else {
    throw std::invalid_argument(
            "ScheduleChangeRecord add failed, type [" + type + "] is invalid."
    );
  }
}

const std::vector<ChangeAction> &
ScheduleChangeRecord::get(
  const std::string & type
) const
{
  if (type == record_data_type::EVENT || type == record_data_type::TASK) {
    return event_changes_;
  } else if (type == record_data_type::PROCESS) {
    return process_changes_;
  } else if (type == record_data_type::SERIES) {
    return series_changes_;
  } else {
    throw std::invalid_argument(
            "ScheduleChangeRecord get failed, type [" + type + "] is invalid."
    );
  }
}

ScheduleChangeRecord ScheduleChangeRecord::squash(
  const std::vector<ScheduleChangeRecord> & records
)
{
  ScheduleChangeRecord result;

  squash_record_by_data_type(record_data_type::EVENT, records, result);
  squash_record_by_data_type(record_data_type::PROCESS, records, result);
  squash_record_by_data_type(record_data_type::SERIES, records, result);
  return result;
}

}  // namespace data
}  // namespace rmf2_scheduler
