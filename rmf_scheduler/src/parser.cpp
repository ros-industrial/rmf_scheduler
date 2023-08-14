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

#include "rmf_scheduler/parser.hpp"

namespace rmf_scheduler
{

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
    std::string series_id, dependency_id, event_details;

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

    if (event_json.contains("event_details")) {
      event_details = event_json["event_details"].dump();
    }

    // Create event
    Event event {
      event_json["description"].get<std::string>(),                         // description
      event_json["type"].get<std::string>(),                                // type
      static_cast<uint64_t>(event_json["start_time"].get<double>() * 1e9),  // start time
      duration,                                                             // duration
      id,                                                                   // id
      series_id,                                                            // series id
      dependency_id,                                                        // dag id
      event_details                                                         // event details
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

void json_to_update_event_time(
  const nlohmann::json & update_event_time_json,
  rmf_scheduler::UpdateEventTime & update_event_time)
{
  update_event_time.id = update_event_time_json["id"].get<std::string>();
  update_event_time.start_time = update_event_time_json["start_time"].get<double>() * 1e9;
  update_event_time.duration = update_event_time_json["duration"].get<double>() * 1e9;
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
    event_json["event_details"] = nlohmann::json::parse(event.event_details);

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


}  // namespace rmf_scheduler
