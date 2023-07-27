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

#include <thread>

#include "rmf_scheduler/scheduler.hpp"
#include "rmf_scheduler/schema_validator.hpp"
#include "rmf_scheduler/error_code.hpp"
#include "rmf_scheduler/system_time_utils.hpp"
#include "rmf_scheduler/parser.hpp"

#include "rmf_scheduler/schemas/schedule.hpp"
#include "rmf_scheduler/schemas/event.hpp"
#include "rmf_scheduler/schemas/dependency.hpp"
#include "rmf_scheduler/schemas/series.hpp"
#include "rmf_scheduler/schemas/get_schedule_request.hpp"
#include "rmf_scheduler/schemas/delete_schedule_request.hpp"

namespace rmf_scheduler
{

namespace utils
{

/// Convert floating point to integer with proper range limit
/**
 * static_cast will cause floating point value higher than integer limit to be 0.
 *
 * \param[in] in floating point value
 * \return integer
 */
uint64_t clamp_cast_uint64_t(double in)
{
  if (in < 0) {
    return 0;
  }

  if (in > static_cast<double>(UINT64_MAX)) {
    return UINT64_MAX;
  }

  return static_cast<uint64_t>(in);
}

}  // namespace utils

Scheduler::Scheduler()
{
  const std::vector<nlohmann::json> schemas = {
    schemas::event,
    schemas::dependency,
    schemas::series,
    schemas::schedule,
    schemas::get_schedule_request,
    schemas::delete_schedule_request,
  };

  schema_validator_ = std::make_unique<SchemaValidator>(schemas);

  last_write_time_ = utils::now();
}

Scheduler::~Scheduler()
{
}

ErrorCode Scheduler::add_schedule(
  const nlohmann::json & json)
{
  Schedule::Description description;
  try {
    auto json_uri = nlohmann::json_uri{
      schemas::schedule["$id"]
    };

    // Validate JSON
    schema_validator_->validate(
      json_uri,
      json);

    // Parse the validated json into description
    parser::json_to_schedule(json, description);
  } catch (const std::exception & e) {
    // Invalid Json schema
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_SCHEMA,
      e.what()
    };
  }

  // Call add_schedule to check logic
  return RMF_SCHEDULER_RESOLVE_ERROR_CODE(add_schedule(description));
}

void Scheduler::add_schedule(const Schedule::Description & schedule)
{
  std::unordered_map<std::string, Event> events_to_add;
  std::unordered_map<std::string, DAG> dags_to_add;
  std::unordered_map<std::string, Series> event_series_map_to_add, dag_series_map_to_add;
  uint64_t read_time;

  {  // Read Lock
    ReadLock lk(mtx_);
    read_time = utils::now();

    // Validate the events
    schedule_.validate_add_events(schedule.events, events_to_add);

    // Validate the dependencies
    schedule_.validate_add_dags(
      schedule.dependencies,
      dags_to_add,
      events_to_add);

    // Validate the series
    schedule_.validate_add_series_map(
      schedule.series_map,
      event_series_map_to_add,
      dag_series_map_to_add,
      events_to_add,
      dags_to_add);
  }  // Read Lock

  // Return early if there is nothing to write
  if (events_to_add.empty() && dags_to_add.empty() &&
    event_series_map_to_add.empty() && dag_series_map_to_add.empty())
  {
    return;
  }

  // Write lock
  WriteLock lk(mtx_);

  // If there is writing after the read, exit immediately
  if (last_write_time_ > read_time) {
    throw ScheduleMultipleWriteException(
            "Schedule already written by %s at time \n\t%s.",
            last_writer_.c_str(), utils::to_localtime(last_write_time_));
  }

  // Update schedule cache
  // Add new events as dangling events
  for (auto & event_itr : events_to_add) {
    schedule_.add_event(event_itr.second);
  }

  // Link the events together using DAG
  for (auto & dag_itr : dags_to_add) {
    schedule_.add_dag(dag_itr.first, std::move(dag_itr.second));

    // Generate the start time for events based on DAG
    schedule_.generate_dag_event_start_time(dag_itr.first);
  }

  // Link the events / DAGs together using series.
  for (auto & series_itr : event_series_map_to_add) {
    schedule_.add_event_series(series_itr.first, std::move(series_itr.second));
  }

  for (auto & series_itr : dag_series_map_to_add) {
    schedule_.add_dag_series(series_itr.first, std::move(series_itr.second));
  }

  // Update who made changes last
  last_write_time_ = utils::now();
  last_writer_ = _thread_id();
}

ErrorCode Scheduler::update_schedule(
  const nlohmann::json & schedule_json)
{
  Schedule::Description description;
  try {
    auto json_uri = nlohmann::json_uri{
      schemas::schedule["$id"]
    };

    // Validate JSON
    schema_validator_->validate(
      json_uri,
      schedule_json);

    // Parse the validated json into description
    parser::json_to_schedule(schedule_json, description);
  } catch (const std::exception & e) {
    // Invalid Json schema
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_SCHEMA,
      e.what()
    };
  }
  return RMF_SCHEDULER_RESOLVE_ERROR_CODE(update_schedule(description));
}

void Scheduler::update_schedule(const Schedule::Description & schedule)
{
  std::unordered_map<std::string, Event> events_to_update;
  std::unordered_map<std::string, DAG> dags_to_update;
  std::unordered_map<std::string, Series> event_series_map_to_update, dag_series_map_to_update;
  uint64_t read_time;

  {  // Read lock
    ReadLock lk(mtx_);
    read_time = utils::now();

    // Validate the events
    schedule_.validate_update_events(schedule.events, events_to_update);

    // Validate the DAGs
    schedule_.validate_update_dags(schedule.dependencies, dags_to_update);

    // Validate the Series
    schedule_.validate_update_series_map(
      schedule.series_map,
      event_series_map_to_update,
      dag_series_map_to_update,
      events_to_update,
      dags_to_update);
  }  // Read Lock

  // Return early if there is nothing to update
  if (events_to_update.empty() && dags_to_update.empty() &&
    event_series_map_to_update.empty() && dag_series_map_to_update.empty())
  {
    return;
  }

  WriteLock lk(mtx_);
  // If there is writing after the read, exit immediately
  if (last_write_time_ > read_time) {
    throw ScheduleMultipleWriteException(
            "Schedule already written by %s at time \n\t%s.",
            last_writer_.c_str(), utils::to_localtime(last_write_time_));
  }

  // Update schedule cache
  // Update events first
  for (auto & event_itr : events_to_update) {
    schedule_.update_event(event_itr.second);
  }

  // Update the dependencies
  for (auto & dag_itr : dags_to_update) {
    schedule_.update_dag(dag_itr.first, std::move(dag_itr.second));

    // Generate the start time for events based on DAG
    schedule_.generate_dag_event_start_time(dag_itr.first);
  }

  // Update series map
  for (auto & series_itr : event_series_map_to_update) {
    schedule_.update_event_series(series_itr.first, std::move(series_itr.second));
  }

  for (auto & series_itr : dag_series_map_to_update) {
    schedule_.update_dag_series(series_itr.first, std::move(series_itr.second));
  }

  // Update who made changes last
  last_write_time_ = utils::now();
  last_writer_ = _thread_id();
}


nlohmann::json Scheduler::get_schedule(
  const nlohmann::json & request_json) const
{
  nlohmann::json response_json;
  uint64_t start_time, end_time;
  try {
    auto json_uri = nlohmann::json_uri{
      schemas::get_schedule_request["$id"]
    };

    // Validate JSON
    schema_validator_->validate(
      json_uri,
      request_json);

    // Ignore value higher than UINT64_MAX
    start_time = request_json.contains("start_time") ?
      utils::clamp_cast_uint64_t(request_json["start_time"].get<double>() * 1e9) : 0;

    end_time = request_json.contains("end_time") ?
      utils::clamp_cast_uint64_t(request_json["end_time"].get<double>() * 1e9) : UINT64_MAX;
  } catch (const std::exception & e) {
    // Return schema errors
    response_json["error_code"] =
      nlohmann::json {
      {"value", ErrorCode::FAILURE | ErrorCode::INVALID_SCHEMA},
      {"detail", e.what()}
    };
    return response_json;
  }

  try {
    auto schedule_description = get_schedule(start_time, end_time);
    parser::schedule_to_json(schedule_description, response_json["schedule"]);
    response_json["error_code"] =
      nlohmann::json {
      {"value", ErrorCode::SUCCESS},
      {"detail", ""}
    };
  } catch (const std::exception & e) {
    // Unknown error
    response_json["error_code"] =
      nlohmann::json {
      {"value", ErrorCode::FAILURE},
      {"detail", e.what()}
    };
    return response_json;
  }
  return response_json;
}

Schedule::Description Scheduler::get_schedule(
  uint64_t start_time,
  uint64_t end_time) const
{
  ReadLock lk(mtx_);
  Schedule::Description schedule_description;
  auto event_vector = schedule_.events_handler_const().lookup_events(start_time, end_time);
  for (auto & event : event_vector) {
    schedule_description.events.emplace(event.id, event);

    // Add Dependency information if needed
    if (!event.dag_id.empty() &&
      schedule_description.dependencies.find(event.dag_id) ==
      schedule_description.dependencies.end())
    {
      schedule_description.dependencies.emplace(
        event.dag_id,
        schedule_.get_dag(event.dag_id).description());
    }

    // Add series information if needed
    if (!event.series_id.empty() &&
      schedule_description.series_map.find(event.series_id) ==
      schedule_description.series_map.end())
    {
      schedule_description.series_map.emplace(
        event.series_id,
        schedule_.get_series(event.series_id).description());
    }
  }
  return schedule_description;
}


ErrorCode Scheduler::delete_schedule(const nlohmann::json & request_json)
{
  std::vector<std::string> event_ids, dependency_ids, series_ids;
  try {
    auto json_uri = nlohmann::json_uri{
      schemas::delete_schedule_request["$id"]
    };

    // Validate JSON
    schema_validator_->validate(
      json_uri,
      request_json);

    // Parse the validated json into list of ids
    if (request_json.contains("event_ids")) {
      for (auto & event_id_itr : request_json["event_ids"]) {
        event_ids.push_back(event_id_itr.get<std::string>());
      }
    }

    if (request_json.contains("dependency_ids")) {
      for (auto & dependency_id_itr : request_json["dependency_ids"]) {
        dependency_ids.push_back(dependency_id_itr.get<std::string>());
      }
    }

    if (request_json.contains("series_ids")) {
      for (auto & series_id_itr : request_json["series_ids"]) {
        series_ids.push_back(series_id_itr.get<std::string>());
      }
    }
  } catch (const std::exception & e) {
    // Invalid Json schema
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_SCHEMA,
      e.what()
    };
  }
  return RMF_SCHEDULER_RESOLVE_ERROR_CODE(
    delete_schedule(event_ids, dependency_ids, series_ids));
}

void Scheduler::delete_schedule(
  const std::vector<std::string> & event_ids,
  const std::vector<std::string> & dependency_ids,
  const std::vector<std::string> & series_ids)
{
  uint64_t read_time;

  std::vector<std::string> event_series_ids, dag_series_ids;

  {  // Read lock
    ReadLock lk(mtx_);
    read_time = utils::now();

    // Validate the events to be deleted
    schedule_.validate_delete_events(event_ids);

    // Validate the dags to be deleted
    schedule_.validate_delete_dags(dependency_ids);

    // Validate the series to be deleted
    schedule_.validate_delete_series_map(
      series_ids,
      event_series_ids, dag_series_ids);
  }  // Read Lock

  // Return early if there is nothing to update
  if (event_ids.empty() && dependency_ids.empty() &&
    event_series_ids.empty() && dag_series_ids.empty())
  {
    return;
  }

  // Write lock
  WriteLock lk(mtx_);
  // If there is writing after the read, exit immediately
  if (last_write_time_ > read_time) {
    throw ScheduleMultipleWriteException(
            "Schedule already written by %s at time \n\t%s.",
            last_writer_.c_str(), utils::to_localtime(last_write_time_));
  }

  // Update schedule cache
  // Delete events from the cache
  // Store dags deleted along the way
  std::unordered_set<std::string> dag_deleted;
  for (auto & event_id : event_ids) {
    Event event = schedule_.get_event(event_id);

    bool is_dag = !event.dag_id.empty();
    bool is_series = !event.series_id.empty();

    if (is_dag) {
      // Special operation if DAG is reoccurring
      if (is_series) {
        // Delete DAG if it only contains the event to be deleted
        auto all_nodes = schedule_.get_dag(event.dag_id).all_nodes();
        if (all_nodes.size() == 1 && all_nodes.front() == event_id) {
          schedule_.delete_dag_series_occurrence(
            event.series_id,
            event.dag_id);
          dag_deleted.emplace(event.dag_id);
        } else {
          // TODO(anyone): Make this cleaner maybe?
          // Detach the event from the DAG before deletion
          DAG new_dag(schedule_.get_dag(event.dag_id).description());
          new_dag.delete_node(event_id);
          // Make this DAG an exception in the series
          schedule_.update_dag_series_occurrence(
            event.series_id,
            event.dag_id,
            new_dag);
          schedule_.update_dag(event.dag_id, std::move(new_dag));
          schedule_.delete_event(event_id);
        }
      } else {
        // Detach the event from the DAG before deletion
        schedule_.detach_dag_event(event.dag_id, event_id);
        schedule_.delete_event(event_id);
      }
    } else if (is_series) {
      // Event series
      schedule_.delete_event_series_occurrence(
        event.series_id,
        event_id);
    } else {
      schedule_.delete_event(event_id);
    }
  }

  // Delete dags from the cache
  for (auto & dag_id : dependency_ids) {
    if (dag_deleted.find(dag_id) != dag_deleted.end()) {
      continue;
    }
    auto all_nodes = schedule_.get_dag(dag_id).all_nodes();
    if (!all_nodes.empty() &&
      !schedule_.get_event(all_nodes.front()).series_id.empty())
    {
      // DAG is part of a series
      auto series_id = schedule_.get_event(all_nodes.front()).series_id;
      schedule_.delete_dag_series_occurrence(
        series_id, dag_id);
    } else {
      schedule_.delete_dag(dag_id);
    }
  }

  for (auto & series_id : event_series_ids) {
    schedule_.delete_event_series(series_id);
  }

  for (auto & series_id : dag_series_ids) {
    schedule_.delete_dag_series(series_id);
  }

  // Purge all empty dags and series
  schedule_.purge();
}

const Schedule & Scheduler::get_schedule_handler_const() const
{
  return schedule_;
}

Schedule & Scheduler::get_schedule_handler()
{
  return schedule_;
}

std::string Scheduler::_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

}  // namespace rmf_scheduler
