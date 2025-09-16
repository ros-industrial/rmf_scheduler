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

#ifndef RMF2_SCHEDULER__DATA__JSON_SERIALIZER_HPP_
#define RMF2_SCHEDULER__DATA__JSON_SERIALIZER_HPP_

#include <map>
#include <memory>
#include <string>
#include <optional>
#include <vector>

#include "nlohmann/json.hpp"
#include "rmf2_scheduler/data/json_macros.hpp"
#include "rmf2_scheduler/data/time.hpp"
#include "rmf2_scheduler/data/duration.hpp"
#include "rmf2_scheduler/data/event.hpp"
#include "rmf2_scheduler/data/task.hpp"
#include "rmf2_scheduler/data/graph.hpp"
#include "rmf2_scheduler/data/process.hpp"
#include "rmf2_scheduler/data/occurrence.hpp"
#include "rmf2_scheduler/data/schedule_action.hpp"
#include "rmf2_scheduler/data/series.hpp"

// Custom Serializer
namespace nlohmann
{

/// JSON serializer for std::optional
template<typename T>
struct adl_serializer<std::optional<T>>
{
  static void from_json(const json & j, std::optional<T> & opt)
  {
    if (!j.is_null()) {
      opt = j.template get<T>();
    } else {
      opt = std::nullopt;
    }
  }

  static void to_json(json & j, const std::optional<T> & opt)
  {
    if (opt.has_value()) {
      j = *opt;
    } else {
      j = nullptr;
    }
  }
};

/// JSON serializer for std::shared_ptr
template<typename T>
struct adl_serializer<std::shared_ptr<T>>
{
  static void from_json(const json & j, std::shared_ptr<T> & opt)
  {
    if (!j.is_null()) {
      opt = std::make_shared<T>(j.template get<T>());
    } else {
      opt = nullptr;
    }
  }

  static void to_json(json & j, const std::shared_ptr<T> & opt)
  {
    if (opt) {
      j = *opt;
    } else {
      j = nullptr;
    }
  }
};

/// JSON serializer for Time
template<>
struct adl_serializer<rmf2_scheduler::data::Time>
{
  static void from_json(const json & j, rmf2_scheduler::data::Time & time)
  {
    time = rmf2_scheduler::data::Time::from_ISOtime(j.template get<std::string>());
  }

  static void to_json(json & j, const rmf2_scheduler::data::Time & time)
  {
    j = time.to_ISOtime();
  }
};

/// JSON serializer for Duration
template<>
struct adl_serializer<rmf2_scheduler::data::Duration>
{
  static void from_json(const json & j, rmf2_scheduler::data::Duration & duration)
  {
    duration = rmf2_scheduler::data::Duration::from_seconds(j.template get<double>());
  }

  static void to_json(json & j, const rmf2_scheduler::data::Duration & duration)
  {
    j = duration.seconds();
  }
};

/// JSON serializer for Graph
template<>
struct adl_serializer<rmf2_scheduler::data::Graph>
{
  static void from_json(const json & j, rmf2_scheduler::data::Graph & graph)
  {
    std::vector<json> graph_v;
    if (j.is_array()) {
      graph_v = j.template get<std::vector<json>>();
    } else {
      graph_v = {j};
    }

    // Add nodes
    for (auto & itr : graph_v) {
      std::string id;
      itr.at("id").get_to(id);
      graph.add_node(id);
    }

    // Add edges
    for (auto & itr : graph_v) {
      std::string destination;
      itr.at("id").get_to(destination);

      std::vector<json> dependencies;
      auto & dependencies_j = itr.at("needs");
      if (dependencies_j.is_array()) {
        dependencies_j.get_to(dependencies);
      } else {
        dependencies = {dependencies_j};
      }
      for (auto & dependency_itr : dependencies) {
        std::string source;
        std::string edge_type;
        dependency_itr.at("id").get_to(source);
        dependency_itr.at("type").get_to(edge_type);
        graph.add_edge(
          source,
          destination,
          rmf2_scheduler::data::Edge(edge_type)
        );
      }
    }
  }

  static rmf2_scheduler::data::Graph from_json(const json & j)
  {
    rmf2_scheduler::data::Graph graph;
    from_json(j, graph);
    return graph;
  }

  static void to_json(json & j, const rmf2_scheduler::data::Graph & graph)
  {
    j = std::vector<json>();
    graph.for_each_node_ordered(
      [&j](rmf2_scheduler::data::Node::Ptr node) {
        json node_info;
        node_info["id"] = node->id();

        std::map<std::string, rmf2_scheduler::data::Edge> ordered_inbound_edges {
          node->inbound_edges().begin(),
          node->inbound_edges().end()
        };
        std::vector<json> dependencies_info;
        for (auto & inbound_edge : ordered_inbound_edges) {
          json dependency_info;
          dependency_info["id"] = inbound_edge.first;
          dependency_info["type"] = inbound_edge.second.type;
          dependencies_info.push_back(dependency_info);
        }
        node_info["needs"] = dependencies_info;
        j.push_back(node_info);
      }
    );
  }
};

/// JSON serializer for Series
template<>
struct adl_serializer<rmf2_scheduler::data::Series>
{
  static void from_json(const json & j, rmf2_scheduler::data::Series & series);

  static rmf2_scheduler::data::Series from_json(const json & j);

  static void to_json(json & j, const rmf2_scheduler::data::Series & series);
};

}  // namespace nlohmann

// Auto-generated serializer
namespace rmf2_scheduler
{

namespace data
{

/// Event serializer
inline void to_json(
  nlohmann::json & event_j,
  const Event & event_t
)
{
  event_j["id"] = event_t.id;
  event_j["type"] = event_t.type;
  event_j["description"] = event_t.description;
  event_j["start_time"] = event_t.start_time;

  // custom handler for end time
  if (!event_t.duration.has_value()) {
    event_j["end_time"] = nullptr;
  } else {
    event_j["end_time"] = event_t.start_time + *event_t.duration;
  }
  event_j["series_id"] = event_t.series_id;
  event_j["process_id"] = event_t.process_id;
}

/// Event deserializer
inline void from_json(
  const nlohmann::json & event_j,
  Event & event_t)
{
  // Required members
  event_t.id = event_j.at("id");
  event_t.type = event_j.at("type");
  event_t.start_time = event_j.at("start_time");

  // Optional members
  Event event_default_obj;
  event_t.description = event_j.value("description", event_default_obj.description);
  event_t.series_id = event_j.value("series_id", event_default_obj.series_id);
  event_t.process_id = event_j.value("process_id", event_default_obj.process_id);

  // custom handler for the end_time
  std::optional<Time> end_time;
  end_time = event_j.value("end_time", end_time);
  if (end_time.has_value()) {
    if (event_t.start_time > *end_time) {
      throw std::underflow_error(
              "from_json failed, event end_time is before start_time."
      );
    }
    event_t.duration = *end_time - event_t.start_time;
  }
}

/// Task serializer
inline void to_json(
  nlohmann::json & task_j,
  const Task & task_t
)
{
  // Serialize base class info
  to_json(task_j, dynamic_cast<const Event &>(task_t));

  task_j["resource_id"] = task_t.resource_id;
  task_j["deadline"] = task_t.deadline;
  task_j["status"] = task_t.status;
  task_j["planned_start_time"] = task_t.planned_start_time;
  task_j["task_details"] = task_t.task_details;

  // custom handler for planned_end_time
  if (!task_t.planned_duration.has_value()) {
    task_j["planned_end_time"] = nullptr;
  } else {
    task_j["planned_end_time"] = *task_t.planned_start_time + *task_t.planned_duration;
  }
  task_j["estimated_duration"] = task_t.estimated_duration;
  task_j["actual_start_time"] = task_t.actual_start_time;

  // custom handler for actual_end_time
  if (!task_t.actual_duration.has_value()) {
    task_j["actual_end_time"] = nullptr;
  } else {
    task_j["actual_end_time"] = *task_t.actual_start_time + *task_t.actual_duration;
  }
}

/// Task deserializer
inline void from_json(
  const nlohmann::json & task_j,
  Task & task_t)
{
  // Deserialize base class info
  from_json(task_j, dynamic_cast<Event &>(task_t));

  // Required members
  task_t.status = task_j.at("status");

  // Optional members
  Task task_default_obj;
  task_t.resource_id = task_j.value("resource_id", task_default_obj.resource_id);
  task_t.deadline = task_j.value("deadline", task_default_obj.deadline);
  task_t.status = task_j.value("status", task_default_obj.status);
  task_t.planned_start_time = task_j.value(
    "planned_start_time",
    task_default_obj.planned_start_time
  );
  task_t.estimated_duration = task_j.value(
    "estimated_duration",
    task_default_obj.estimated_duration
  );
  task_t.actual_start_time = task_j.value(
    "actual_start_time",
    task_default_obj.actual_start_time
  );
  task_t.task_details = task_j.value("task_details", task_default_obj.task_details);

  // custom handler for the planned_end_time
  std::optional<Time> planned_end_time;
  planned_end_time = task_j.value("planned_end_time", planned_end_time);
  if (planned_end_time.has_value()) {
    if (!task_t.planned_start_time.has_value()) {
      throw std::underflow_error(
              "from_json failed, task planned_end_time is defined but planned_start_time is null."
      );
    }
    if (*task_t.planned_start_time > planned_end_time) {
      throw std::underflow_error(
              "from_json failed, task planned_end_time is before planned_start_time."
      );
    }
    task_t.planned_duration = *planned_end_time - *task_t.planned_start_time;
  }

  // custom handler for the actual_end_time
  std::optional<Time> actual_end_time;
  actual_end_time = task_j.value("actual_end_time", actual_end_time);
  if (actual_end_time.has_value()) {
    Time start_time = task_t.start_time;
    if (!task_t.actual_start_time.has_value()) {
      start_time = *task_t.actual_start_time;
    }
    if (*task_t.actual_start_time > actual_end_time) {
      throw std::underflow_error(
              "from_json failed, task actual_end_time is before actual_start_time."
      );
    }
    task_t.actual_duration = *actual_end_time - *task_t.actual_start_time;
  }
}

/// Process serializer
RS_JSON_DESERIALIZER_DEFINE_TYPE(
  Process,
  RS_JSON_DESERIALIZER_REQUIRED_MEMBERS(
    id,
    graph
  )
  RS_JSON_DESERIALIZER_OPTIONAL_MEMBERS(
    status,
    current_events,
    process_details
  )
)

RS_JSON_SERIALIZER_DEFINE_TYPE(
  Process,
  id,
  graph,
  status,
  current_events,
  process_details
)

/// Occurrence serializer
RS_JSON_DEFINE(
  Occurrence,
  id,
  time
)

/// ScheduleAction serializer
RS_JSON_SERIALIZER_DEFINE_TYPE(
  ScheduleAction,
  type,
  id,
  event,
  task,
  process,
  node_id,
  source_id,
  destination_id,
  edge_type
)

RS_JSON_DESERIALIZER_DEFINE_TYPE(
  ScheduleAction,
  RS_JSON_DESERIALIZER_REQUIRED_MEMBERS(
    type
  ),
  RS_JSON_DESERIALIZER_OPTIONAL_MEMBERS(
    id,
    event,
    task,
    process,
    node_id,
    source_id,
    destination_id,
    edge_type
  )
)

}  // namespace data
}  // namespace rmf2_scheduler


// adl_serializer implementation
namespace nlohmann
{

// Series
inline void adl_serializer<rmf2_scheduler::data::Series>::from_json(
  const json & j,
  rmf2_scheduler::data::Series & series
)
{
  std::string id;
  std::string type;
  std::vector<rmf2_scheduler::data::Occurrence> occurrences;
  std::string cron;
  std::string timezone;
  rmf2_scheduler::data::Time until = rmf2_scheduler::data::Time::max();
  std::vector<std::string> exception_ids = {};

  j.at("id").get_to(id);
  j.at("type").get_to(type);
  j.at("occurrences").get_to(occurrences);
  j.at("cron").get_to(cron);
  j.at("timezone").get_to(timezone);
  if (j.contains("until")) {
    j.at("until").get_to(until);
  }
  if (j.contains("exception_ids")) {
    j.at("exception_ids").get_to(exception_ids);
  }

  series = rmf2_scheduler::data::Series(
    id,
    type,
    occurrences,
    cron,
    timezone,
    until,
    exception_ids
  );
}

inline rmf2_scheduler::data::Series
adl_serializer<rmf2_scheduler::data::Series>::from_json(const json & j)
{
  rmf2_scheduler::data::Series series;
  from_json(j, series);
  return series;
}

inline void adl_serializer<rmf2_scheduler::data::Series>::to_json(
  json & j,
  const rmf2_scheduler::data::Series & series
)
{
  j["id"] = series.id();
  j["type"] = series.type();
  j["occurrences"] = series.occurrences();
  j["cron"] = series.cron();
  j["timezone"] = series.tz();
  if (series.until() != rmf2_scheduler::data::Time::max()) {
    j["until"] = series.until();
  }
  if (!series.exception_ids().empty()) {
    j["exception_ids"] = series.exception_ids_ordered();
  }
}

}  // namespace nlohmann

#endif  // RMF2_SCHEDULER__DATA__JSON_SERIALIZER_HPP_
