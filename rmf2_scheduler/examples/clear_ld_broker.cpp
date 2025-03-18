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

#include <iostream>
#include <random>

#include "rmf2_scheduler/log.hpp"
#include "rmf2_scheduler/data/uuid.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"
#include "rmf2_scheduler/utils/url_utils.hpp"
#include "rmf2_scheduler/http/transport.hpp"
#include "rmf2_scheduler/storage/schedule_stream_ld_broker.hpp"
#include "rmf2_scheduler/cache/event_action.hpp"
#include "rmf2_scheduler/cache/process_action.hpp"

namespace rs = rmf2_scheduler;

int main()
{
  // rmf2_scheduler::log::setLogLevel(rmf2_scheduler::LogLevel::DEBUG);

  // Context broker URL
  std::string ld_broker_domain = "http://localhost:9090";
  std::string ld_broker_path = "ngsi-ld";

  // Create URL
  std::string ld_broker_url = rs::url::combine(
    ld_broker_domain,
    ld_broker_path
  );

  auto transport = rs::http::Transport::create_default();
  rs::storage::ScheduleStream::Ptr storage_stream =
    std::make_shared<rs::storage::ld_broker::ScheduleStream>(
    "http://localhost:9090/ngsi-ld",
    transport
    );

  auto schedule_cache = rs::cache::ScheduleCache::make_shared();
  std::string error;

  // Read all schedule
  rs::data::TimeWindow time_window;
  time_window.start = rs::data::Time(0);
  time_window.end = rs::data::Time::max();

  bool result = storage_stream->read_schedule(
    schedule_cache,
    time_window,
    error
  );

  if (!result) {
    LOG_ERROR("Unable to load schedule: %s.", error.c_str());
    return 1;
  }

  // Create delete actions
  std::vector<rs::cache::Action::Ptr> actions;
  std::vector<rs::data::ScheduleChangeRecord> records;

  // Add process delete actions
  for (auto & process : schedule_cache->get_all_processes()) {
    auto action = rs::cache::ProcessAction::make_shared(
      rs::data::action_type::PROCESS_DELETE_ALL,
      rs::cache::ActionPayload().id(process->id)
    );
    LOG_INFO("deleting process: %s", process->id.c_str());
    actions.push_back(action);
  }

  for (auto & action : actions) {
    bool result = action->validate(schedule_cache, error);
    if (!result) {
      LOG_ERROR("Unable to delete process: %s", error.c_str());
      return 1;
    }

    // Apply
    action->apply();
    records.push_back(action->record());
  }

  actions.clear();

  // Add event delete actions
  for (auto & event : schedule_cache->get_all_events()) {
    auto action = rs::cache::EventAction::make_shared(
      rs::data::action_type::EVENT_DELETE,
      rs::cache::ActionPayload().id(event->id)
    );
    actions.push_back(action);
  }

  for (auto & action : actions) {
    bool result = action->validate(schedule_cache, error);
    if (!result) {
      LOG_ERROR("Unable to delete dangling event: %s", error.c_str());
      return 1;
    }

    // Apply
    action->apply();
    records.push_back(action->record());
  }

  // Check records
  auto squashed_record = rs::data::ScheduleChangeRecord::squash(records);
  LOG_INFO("Event change records:");
  for (auto & change_action : squashed_record.get(rs::data::record_data_type::EVENT)) {
    LOG_INFO(
      "%s [%s]",
      change_action.id.c_str(),
      change_action.action.c_str()
    );
  }

  LOG_INFO("Process change records:");
  for (auto & change_action : squashed_record.get(rs::data::record_data_type::PROCESS)) {
    LOG_INFO(
      "%s [%s]",
      change_action.id.c_str(),
      change_action.action.c_str()
    );
  }

  LOG_INFO("Update stream");
  result = storage_stream->write_schedule(schedule_cache, records, error);
  if (!result) {
    LOG_ERROR("Failed to write to storage: %s", error.c_str());
    return 1;
  }

  LOG_INFO("Done");
}
