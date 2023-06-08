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

#include "rmf_scheduler/schemas/schedule.hpp"
#include "rmf_scheduler/schemas/event.hpp"
#include "rmf_scheduler/schemas/dependency.hpp"
#include "rmf_scheduler/schemas/series.hpp"
#include "rmf_scheduler/schemas/get_schedule_request.hpp"

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

namespace parser
{

void json_to_events(
  const nlohmann::json & events_json,
  std::unordered_map<std::string, Event> & events_description)
{
  for (auto & itr : events_json.items()) {
    auto & event_json = itr.value();

    // Convert event json
    std::string id = itr.key();

    uint64_t duration = 0;
    std::string series_id, dependency_id;

    // Check for optional items first
    if (event_json.contains("duration")) {
      duration = static_cast<uint64_t>(event_json["duration"].get<double>() * 1e9);
    }

    if (event_json.contains("series_id")) {
      series_id = event_json["series_id"];
    }

    if (event_json.contains("dependency_id")) {
      dependency_id = event_json["dependency_id"];
    }

    // Create event
    Event event {
      event_json["description"].get<std::string>(),                         // description
      event_json["type"].get<std::string>(),                                // type
      static_cast<uint64_t>(event_json["start_time"].get<double>() * 1e9),  // start time
      duration,                                                             // duration
      id,                                                                   // id
      series_id,                                                            // series id
      dependency_id                                                         // dag id
    };
    events_description.emplace(id, std::move(event));
  }
}

void json_to_dependencies(
  const nlohmann::json & dependencies_json,
  std::unordered_map<std::string, DAG::Description> & dependencies_description)
{
  for (auto & dependency_itr : dependencies_json.items()) {
    auto & dependency_json = dependency_itr.value();

    // Convert DAG
    DAG::Description dag_description;
    for (auto dependency_itr : dependency_json.items()) {
      dag_description.emplace(
        dependency_itr.key(),
        dependency_itr.value().get<DAG::DependencyInfo>()
      );
    }
    dependencies_description.emplace(dependency_itr.key(), std::move(dag_description));
  }
}

void json_to_series_map(
  const nlohmann::json & series_json,
  std::unordered_map<std::string, Series::Description> & series_map_description)
{
  for (auto & series_itr : series_json.items()) {
    auto & series_json = series_itr.value();

    // Convert series json
    Series::Description series_description;
    series_description.cron = series_json["cron"].get<std::string>();
    series_description.timezone = series_json["timezone"].get<std::string>();
    for (auto & occurrence_itr : series_json["occurrences"]) {
      Series::Occurrence occurrence {
        0,                                // time need to look up from events
        occurrence_itr.get<std::string>()  // id
      };
      series_description.occurrences.push_back(occurrence);
    }

    // Convert optional items
    if (series_json.contains("until")) {
      series_description.until =
        static_cast<uint64_t>(series_json["until"].get<double>() * 1e9);
    } else {
      series_description.until = UINT64_MAX;
    }

    if (series_json.contains("id_prefix")) {
      series_description.id_prefix =
        series_json["id_prefix"].get<std::string>();
    }

    if (series_json.contains("exceptions")) {
      for (auto & exception_itr : series_json["exceptions"]) {
        series_description.exception_ids.emplace_back(
          exception_itr.get<std::string>());
      }
    }

    series_map_description.emplace(series_itr.key(), std::move(series_description));
  }
}

void json_to_schedule(
  const nlohmann::json & schedule_json,
  Schedule::Description & schedule_description)
{
  // Convert required fields: events
  json_to_events(
    schedule_json["events"],
    schedule_description.events);

  // Convert optional fields
  if (schedule_json.contains("dependencies")) {
    json_to_dependencies(
      schedule_json["dependencies"],
      schedule_description.dependencies);
  }

  if (schedule_json.contains("series")) {
    json_to_series_map(
      schedule_json["series"],
      schedule_description.series_map);
  }
}

void events_to_json(
  const std::unordered_map<std::string, Event> & events_description,
  nlohmann::json & events_json)
{
  // return empty instead of null
  if (events_description.empty()) {
    events_json = nlohmann::json({});
    return;
  }

  for (auto & itr : events_description) {
    const Event & event = itr.second;
    nlohmann::json event_json;
    event_json["description"] = event.description;
    event_json["type"] = event.type;
    event_json["start_time"] = 1e-9 * event.start_time;

    // TODO(Briancbn): event_details
    event_json["event_details"] = nlohmann::json({});

    // Optional items
    if (event.duration > 0) {
      event_json["duration"] = 1e-9 * event.duration;
    }

    if (!event.series_id.empty()) {
      event_json["series_id"] = event.series_id;
    }

    if (!event.dag_id.empty()) {
      event_json["dependency_id"] = event.dag_id;
    }

    events_json[itr.first] = event_json;
  }
}

void dependencies_to_json(
  const std::unordered_map<std::string, DAG::Description> & dependencies_description,
  nlohmann::json & dependencies_json)
{
  for (auto itr : dependencies_description) {
    nlohmann::json dependency_graph_json;
    for (auto itr_d : itr.second) {
      dependency_graph_json[itr_d.first] = itr_d.second;
    }
    dependencies_json[itr.first] = dependency_graph_json;
  }
}

void series_map_to_json(
  const std::unordered_map<std::string, Series::Description> & series_map_description,
  nlohmann::json & series_map_json)
{
  for (auto itr : series_map_description) {
    nlohmann::json series_json;
    const Series::Description & series = itr.second;
    series_json["cron"] = series.cron;
    series_json["timezone"] = series.timezone;

    std::vector<std::string> occurrences_wo_starttime;
    for (auto occurrence : series.occurrences) {
      occurrences_wo_starttime.push_back(occurrence.id);
    }
    series_json["occurrences"] = occurrences_wo_starttime;

    // Optional items
    if (series.until != UINT64_MAX) {
      series_json["until"] = series.until;
    }
    if (!series.id_prefix.empty()) {
      series_json["id_prefix"] = series.id_prefix;
    }
    if (!series.exception_ids.empty()) {
      series_json["exceptions"] = series.exception_ids;
    }
    series_map_json[itr.first] = series_json;
  }
}

void schedule_to_json(
  const Schedule::Description & schedule_description,
  nlohmann::json & schedule_json)
{
  // Convert events
  events_to_json(
    schedule_description.events,
    schedule_json["events"]);

  // Convert optional fields
  if (!schedule_description.dependencies.empty()) {
    dependencies_to_json(
      schedule_description.dependencies,
      schedule_json["dependencies"]);
  }

  if (!schedule_description.series_map.empty()) {
    series_map_to_json(
      schedule_description.series_map,
      schedule_json["series"]);
  }
}

}  // namespace parser

Scheduler::Scheduler()
{
  const std::vector<nlohmann::json> schemas = {
    schemas::event,
    schemas::dependency,
    schemas::series,
    schemas::schedule,
    schemas::get_schedule_request,
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
  return add_schedule(description);
}

ErrorCode Scheduler::add_schedule(const Schedule::Description & schedule)
{
  return
    _resolve_error_code(
    [ = ]() {
      _add_schedule(schedule);
    }
    );
}

void Scheduler::_add_schedule(const Schedule::Description & schedule)
{
  const auto & temp_dags_description = schedule.dependencies;
  auto temp_series_description = schedule.series_map;
  auto temp_events = schedule.events;

  std::unordered_map<std::string, DAG> temp_dags;
  std::unordered_map<std::string, Series> temp_series_map;
  uint64_t read_time;

  {  // Read Lock
    ReadLock lk(mtx_);
    read_time = utils::now();

    // Validate the events
    for (auto event_itr : temp_events) {
      // throw error if the events already exists
      if (schedule_.eh.has_event(event_itr.first)) {
        throw EventsHandlerIDException(
                event_itr.first.c_str(),
                "add_schedule error "
                "Event [%s] already exists, use a different ID",
                event_itr.first.c_str());
      }
    }

    // Validate the dependencies
    for (auto & dag_itr : temp_dags_description) {
      // throw error if DAG graph ID overlaps with an existing one
      if (schedule_.dags.find(dag_itr.first) != schedule_.dags.end()) {
        throw DAGIDException(
                dag_itr.first.c_str(),
                "add_schedule error "
                "DAG graph [%s] already exists, use a different ID",
                dag_itr.first.c_str());
      }

      // Validate that All IDs in the dags exist in newly added events
      for (auto & dep_info_itr : dag_itr.second) {
        const auto & event_id = dep_info_itr.first;

        auto event_itr = temp_events.find(event_id);

        if (event_itr == temp_events.end()) {
          throw DAGIDException(
                  event_id.c_str(),
                  "add_schedule error "
                  "Event ID [%s] in dependency graph doesn't exist",
                  event_id.c_str());
        }
        // Check if event has the correct DAG id
        if (event_itr->second.dag_id.empty()) {
          event_itr->second.dag_id = dag_itr.first;
        } else if (event_itr->second.dag_id != dag_itr.first) {
          // Overwrite the event DAG id here
          event_itr->second.dag_id = dag_itr.first;
        }

        for (auto & dependant_id : dep_info_itr.second) {
          if (temp_events.find(dependant_id) == temp_events.end()) {
            throw DAGIDException(
                    dependant_id.c_str(),
                    "add_schedule error "
                    "Event ID [%s] in dependency graph doesn't exist",
                    dependant_id.c_str());
          }
        }
      }
      // Check if DAGs are cyclic
      DAG temp_dag;
      temp_dag = DAG(dag_itr.second, true);  // Throw DAGCyclicException or DAGIDException

      // Everything valid
      temp_dags.emplace(dag_itr.first, std::move(temp_dag));
    }

    // Validate all ids in the series exists in the events or DAG
    for (auto & series_itr : temp_series_description) {
      // First check if the Series ID overlaps with an existing one
      if (schedule_.series_map.find(series_itr.first) != schedule_.series_map.end()) {
        throw SeriesIDException(
                series_itr.first.c_str(),
                "add_schedule error "
                "Series [%s] already exists, use a different ID",
                series_itr.first.c_str());
      }

      // Check if the occurrences exists
      for (auto & occurrence_itr : series_itr.second.occurrences) {
        if (temp_events.find(occurrence_itr.id) != temp_events.end()) {
          // Occurrence is an event
          // Attach time to the occurrence
          auto event_itr = temp_events.find(occurrence_itr.id);
          occurrence_itr.time = event_itr->second.start_time;
        } else if (temp_dags.find(occurrence_itr.id) != temp_dags.end()) {
          // Occurrence is a DAG
          // Attach the first event's start time to the series
          auto dag_itr = temp_dags.find(occurrence_itr.id);
          auto entry_nodes = dag_itr->second.entry_nodes();
          auto earliest_node = std::min_element(
            entry_nodes.begin(), entry_nodes.end(),
            [&temp_events](const std::string & node1, const std::string & node2) -> bool
            {
              return temp_events.at(node1).start_time < temp_events.at(node2).start_time;
            });
          occurrence_itr.time = temp_events.at(*earliest_node).start_time;

          // Update series id of all events under the dag
          auto all_nodes = dag_itr->second.all_nodes();
          for (auto node : all_nodes) {
            temp_events.at(node).series_id = series_itr.first;
          }
        } else {
          // Occurrence doesn't exist
          throw SeriesIDException(
                  occurrence_itr.id.c_str(),
                  "Occurrence [%s] in series [%s] is neither an event nor a dependency graph",
                  occurrence_itr.id.c_str(), series_itr.first.c_str());
        }
      }
      // Validate crons
      // throw SeriesEmptyException or SeriesInvalidCronException
      Series temp_series(series_itr.second);
      temp_series_map.emplace(series_itr.first, std::move(temp_series));
    }
  }  // Read Lock

  // Return early if there is nothing to write
  if (temp_events.empty() && temp_dags.empty() && temp_series_map.empty()) {
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
  // Add new schedule to the cache
  for (auto & dag_itr : temp_dags) {
    schedule_.dags.emplace(dag_itr.first, std::move(dag_itr.second));
  }
  for (auto & series_itr : temp_series_map) {
    schedule_.series_map.emplace(series_itr.first, std::move(series_itr.second));
  }
  for (auto & event_itr : temp_events) {
    schedule_.eh.add_event(event_itr.second);
  }

  // Update who made changes last
  last_write_time_ = utils::now();
  last_writer_ = _thread_id();
}

ErrorCode Scheduler::update_schedule(
  const std::string & schedule_json)
{
  Schedule::Description description;
  try {
    nlohmann::json json;
    json = nlohmann::json::parse(schedule_json);
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
  return update_schedule(description);
}

ErrorCode Scheduler::update_schedule(const Schedule::Description & schedule)
{
  return
    _resolve_error_code(
    [ = ]() {
      _update_schedule(schedule);
    }
    );
}

void Scheduler::_update_schedule(const Schedule::Description & schedule)
{
  const auto & temp_dags_description = schedule.dependencies;
  auto temp_series_description = schedule.series_map;
  auto temp_events = schedule.events;

  std::unordered_map<std::string, DAG> temp_dags;
  std::unordered_map<std::string, Series> temp_series_map;
  uint64_t read_time;

  {  // Read lock
    ReadLock lk(mtx_);
    read_time = utils::now();

    // Validate the events
    for (auto event_itr : temp_events) {
      // throw error if the events DOESN'T exists
      if (!schedule_.eh.has_event(event_itr.first)) {
        throw EventsHandlerIDException(
                event_itr.first.c_str(),
                "update_schedule error "
                "Event [%s] doesn't exists, use a different ID",
                event_itr.first.c_str());
      }
    }


    for (auto & dag_itr : temp_dags_description) {
      // throw error if DAG graph doesn't exist
      if (schedule_.dags.find(dag_itr.first) == schedule_.dags.end()) {
        throw DAGIDException(
                dag_itr.first.c_str(),
                "update_schedule error "
                "DAG graph [%s] doesn't, use a different ID",
                dag_itr.first.c_str());
      }

      // Validate that All IDs in the dags exist in existing events
      for (auto & dep_info_itr : dag_itr.second) {
        const auto & event_id = dep_info_itr.first;
        if (!schedule_.eh.has_event(event_id)) {
          throw DAGIDException(
                  event_id.c_str(),
                  "update_schedule error "
                  "Event ID [%s] in dependency graph doesn't exist",
                  event_id.c_str());
        }

        for (auto & dependant_id : dep_info_itr.second) {
          if (!schedule_.eh.has_event(dependant_id)) {
            throw DAGIDException(
                    dependant_id.c_str(),
                    "update_schedule error "
                    "Event ID [%s] in dependency graph doesn't exist",
                    dependant_id.c_str());
          }
        }
      }

      // Check if DAGs are cyclic
      DAG temp_dag;
      temp_dag = DAG(dag_itr.second, true);  // Throw DAGCyclicException or DAGIDException

      // Everything valid
      temp_dags.emplace(dag_itr.first, std::move(temp_dag));
    }

    // Validate all ids in the series exists in the events or DAG
    for (auto & series_itr : temp_series_description) {
      // Throw error if the Series ID is not an existing one
      if (schedule_.series_map.find(series_itr.first) == schedule_.series_map.end()) {
        throw SeriesIDException(
                series_itr.first.c_str(),
                "update_schedule error "
                "Series [%s] doesn't exist, use a different ID",
                series_itr.first.c_str());
      }

      // Check if the occurrences exists
      for (auto & occurrence_itr : series_itr.second.occurrences) {
        if (schedule_.eh.has_event(occurrence_itr.id)) {
          // Occurrence is an event
          // Attach the first event's start time to the series
          // Check if the event start time will be updated,
          // if so the newly edited event start time will be used instead.
          Event event = schedule_.eh.get_event(occurrence_itr.id);
          auto temp_event_itr = temp_events.find(occurrence_itr.id);
          if (temp_event_itr != temp_events.end() &&
            temp_event_itr->second.start_time != event.start_time)
          {
            occurrence_itr.time = temp_event_itr->second.start_time;
          } else {
            occurrence_itr.time = event.start_time;
          }
        } else if (schedule_.dags.find(occurrence_itr.id) != schedule_.dags.end()) {
          // Occurrence is a DAG
          // Attach the first event's start time to the series
          auto dag_itr = schedule_.dags.find(occurrence_itr.id);
          auto entry_nodes = dag_itr->second.entry_nodes();

          // Iterate through the entry nodes to find the earliest start time
          uint64_t earliest_start_time = UINT64_MAX;
          for (auto event_id : entry_nodes) {
            auto temp_event_itr = temp_events.find(event_id);
            uint64_t start_time;
            if (temp_event_itr != temp_events.end()) {
              start_time = temp_event_itr->second.start_time;
            } else {
              start_time = schedule_.eh.get_event(event_id).start_time;
            }
            if (start_time < earliest_start_time) {
              earliest_start_time = start_time;
            }
          }
          occurrence_itr.time = earliest_start_time;

        } else {
          // Occurrence doesn't exist
          throw SeriesIDException(
                  occurrence_itr.id.c_str(),
                  "Occurrence [%s] in series [%s] is neither an event nor a dependency graph",
                  occurrence_itr.id.c_str(), series_itr.first.c_str());
        }
      }
      // Validate crons
      // throw SeriesEmptyException or SeriesInvalidCronException
      Series temp_series(series_itr.second);
      temp_series_map.emplace(series_itr.first, std::move(temp_series));
    }
  }  // Read Lock

  // Return early if there is nothing to update
  if (temp_events.empty() && temp_dags.empty() && temp_series_map.empty()) {
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
  for (auto & event_itr : temp_events) {
    schedule_.eh.update_event(event_itr.second);
  }

  // Update the dependencies
  for (auto & dag_itr : temp_dags) {
    std::string dag_id = dag_itr.first;
    schedule_.dags[dag_id] = std::move(dag_itr.second);

    // Iterate through to update dag_id for each event
    auto all_nodes = schedule_.dags.at(dag_id).all_nodes();
    for (auto event_id : all_nodes) {
      Event event = schedule_.eh.get_event(event_id);
      if (event.dag_id != dag_id) {
        event.dag_id = dag_id;
        schedule_.eh.update_event(event);
      }
    }
  }
  for (auto & series_itr : temp_series_map) {
    std::string series_id = series_itr.first;
    schedule_.series_map[series_id] = std::move(series_itr.second);
    // Iterate through to update series_id for each event
    auto all_occurences = schedule_.series_map.at(series_id).description().occurrences;
    for (auto occurrence : all_occurences) {
      if (schedule_.eh.has_event(occurrence.id)) {
        // Occurence is an event
        Event event = schedule_.eh.get_event(occurrence.id);
        if (event.series_id != occurrence.id) {
          event.series_id = occurrence.id;
          schedule_.eh.update_event(event);
        }
      } else {
        // Occurence is an DAG
        auto all_nodes = schedule_.dags.at(occurrence.id).all_nodes();
        for (auto event_id : all_nodes) {
          Event event = schedule_.eh.get_event(event_id);
          if (event.series_id != occurrence.id) {
            event.series_id = occurrence.id;
            schedule_.eh.update_event(event);
          }
        }
      }
    }
  }

  // Update who made changes last
  last_write_time_ = utils::now();
  last_writer_ = _thread_id();
}

std::string Scheduler::get_schedule(
  const nlohmann::json & request_json,
  int indent) const
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
    return response_json.dump();
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
  }
  return response_json.dump(indent);
}

Schedule::Description Scheduler::get_schedule(
  uint64_t start_time,
  uint64_t end_time) const
{
  ReadLock lk(mtx_);
  Schedule::Description schedule_description;
  auto event_vector = schedule_.eh.lookup_events(start_time, end_time);
  for (auto & event : event_vector) {
    schedule_description.events.emplace(event.id, event);

    // Add Dependency information if needed
    if (!event.dag_id.empty() &&
      schedule_description.dependencies.find(event.dag_id) ==
      schedule_description.dependencies.end())
    {
      schedule_description.dependencies.emplace(
        event.dag_id,
        schedule_.dags.at(event.dag_id).description());
    }

    // Add series information if needed
    if (!event.series_id.empty() &&
      schedule_description.series_map.find(event.series_id) ==
      schedule_description.series_map.end())
    {
      schedule_description.series_map.emplace(
        event.series_id,
        schedule_.series_map.at(event.series_id).description());
    }
  }
  return schedule_description;
}

ErrorCode Scheduler::_resolve_error_code(std::function<void()> func)
{
  try {
    func();
  } catch (const EventsHandlerIDException & e) {
    // Event ID error
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_ID |
      ErrorCode::INVALID_EVENT,
      e.what()
    };
  } catch (const DAGIDException & e) {
    // DAG ID error
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_ID |
      ErrorCode::INVALID_DEPENDENCY,
      e.what()
    };
  } catch (const DAGCyclicException & e) {
    // DAG Cyclic error
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_LOGIC |
      ErrorCode::INVALID_DEPENDENCY,
      e.what()
    };
  } catch (const SeriesIDException & e) {
    // DAG Cyclic error
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_ID |
      ErrorCode::INVALID_SERIES,
      e.what()
    };
  } catch (const SeriesEmptyException & e) {
    // Series empty
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_LOGIC |
      ErrorCode::INVALID_SERIES,
      e.what()
    };
  } catch (const SeriesInvalidCronException & e) {
    // Series timing invalid
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_LOGIC |
      ErrorCode::INVALID_SERIES,
      e.what()
    };
  } catch (const ScheduleMultipleWriteException & e) {
    // Multiple writer error
    return {
      ErrorCode::FAILURE |
      ErrorCode::MULTIPLE_ACCESS |
      ErrorCode::NO_FIELD,
      e.what()
    };
  } catch (const std::exception & e) {
    // Other errors
    return {
      ErrorCode::FAILURE,
      e.what()
    };
  }
  return ErrorCode::SUCCESS;
}

std::string Scheduler::_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

}  // namespace rmf_scheduler
