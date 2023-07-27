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

#ifndef RMF_SCHEDULER__SCHEDULE_HPP_
#define RMF_SCHEDULER__SCHEDULE_HPP_

#include <string>
#include <vector>
#include <unordered_map>

#include "rmf_scheduler/events_handler.hpp"
#include "rmf_scheduler/dag.hpp"
#include "rmf_scheduler/series.hpp"

namespace rmf_scheduler
{

/// Schedule class
class Schedule
{
public:
  struct Description
  {
    std::unordered_map<std::string, Event> events;
    std::unordered_map<std::string, DAG::Description> dependencies;
    std::unordered_map<std::string, Series::Description> series_map;
  };
  // Dangling event CRUD
  /// Add dangling event
  void add_event(const Event & event);

  /// Update event
  void update_event(const Event & event);

  /// Delete event
  void delete_event(const std::string & event_id);

  /// Get event
  Event get_event(const std::string & event_id) const;

  // DAG CRUD
  /// Add DAG graph for dangling events
  void add_dag(
    const std::string & dag_id,
    DAG && dag);

  /// DAG add additional dependency to an event (within or outside the graph)
  void add_dag_dependency(
    const std::string & dag_id,
    const std::string & event_id,
    const DAG::DependencyInfo & dependency_info);

  /// Update an existing dependency graph
  void update_dag(
    const std::string & dag_id,
    DAG && dag);

  /// Remove an event from the dag
  void detach_dag_event(
    const std::string & dag_id,
    const std::string & event_id);

  /// Delete dag and all events within the DAG
  void delete_dag(const std::string & dag_id);

  /// Automatically calculate start time of every event within a DAG based on dependency.
  void generate_dag_event_start_time(const std::string & dag_id);

  /// Get dag
  const DAG & get_dag(const std::string & dag_id) const;

  /// Get the start time of a DAG based on the earliest event
  uint64_t get_dag_start_time(
    const DAG & dag,
    const std::unordered_map<std::string, Event> & temp_events = {}) const;

  // Event Series CRUD
  /// Add series based on dangling event
  void add_event_series(
    const std::string & series_id,
    Series && series);

  /// Update event series
  void update_event_series(
    const std::string & series_id,
    Series && series);

  /// Expand event series and add additional events
  void expand_event_series_until(
    const std::string & series_id,
    uint64_t time);

  /// Update event series CRON from a specific event
  void update_event_series_cron(
    const std::string & series_id,
    const Event & new_event,
    const std::string & new_cron,
    const std::string & timezone);

  /// Update event series until time
  void update_event_series_until(
    const std::string & series_id,
    uint64_t time);

  /// Update a single event occurrence
  void update_event_series_occurrence(
    const std::string & series_id,
    const Event & new_event);

  /// Delete event occurrence from series
  void delete_event_series_occurrence(
    const std::string & series_id,
    const std::string & event_occurrence);

  /// Delete a series with all its events
  void delete_event_series(
    const std::string & series_id);

  // DAG Series CRUD
  /// Add series based on dangling event
  void add_dag_series(
    const std::string & series_id,
    Series && series);

  /// Update dag series
  void update_dag_series(
    const std::string & series_id,
    Series && series);

  /// Expand dag series and add additional DAGs
  void expand_dag_series_until(
    const std::string & series_id,
    uint64_t time);

  /// Update dag series CRON from a specific DAG
  void update_dag_series_cron(
    const std::string & series_id,
    const std::string & dag_id,
    const DAG & new_dag,
    const std::string & new_cron,
    const std::string & timezone);

  /// Update dag series until time
  void update_dag_series_until(
    const std::string & series_id,
    uint64_t time);

  /// Update a single DAG occurrence to make an exception
  void update_dag_series_occurrence(
    const std::string & series_id,
    const std::string & dag_id,
    const DAG & new_dag);

  /// Delete event occurrence from series
  void delete_dag_series_occurrence(
    const std::string & series_id,
    const std::string & dag_id);

  /// Delete a series with all its events
  void delete_dag_series(
    const std::string & series_id);

  /// Get Series
  const Series & get_series(
    const std::string & series_id) const;

  /// Purge empty DAGs and series
  void purge();

  /// Validate add dangling event operation
  void validate_add_events(
    const std::unordered_map<std::string, Event> & events,
    std::unordered_map<std::string, Event> & valid_events) const;

  /// Validate update event operation
  void validate_update_events(
    const std::unordered_map<std::string, Event> & events,
    std::unordered_map<std::string, Event> & valid_events) const;

  /// Validate delete event operation
  void validate_delete_events(
    const std::vector<std::string> & event_ids) const;

  /// Validate add dangling event operation
  void validate_add_dags(
    const std::unordered_map<std::string, DAG::Description> & dags,
    std::unordered_map<std::string, DAG> & valid_dags,
    const std::unordered_map<std::string, Event> & temp_events = {}) const;

  /// Validate update event operation
  void validate_update_dags(
    const std::unordered_map<std::string, DAG::Description> & dags,
    std::unordered_map<std::string, DAG> & valid_dags) const;

  void validate_dag(
    const DAG::Description & dag_description,
    DAG & valid_dag,
    const std::unordered_map<std::string, Event> & temp_events = {}) const;

  /// Validate delete DAG operation
  void validate_delete_dags(
    const std::vector<std::string> & dag_ids) const;

  bool is_dag_series(
    const std::vector<Series::Occurrence> & occurrences,
    const std::unordered_map<std::string, Event> & temp_events = {},
    const std::unordered_map<std::string, DAG> & temp_dags = {}) const;

  /// Validate add dangling event operation
  void validate_add_series_map(
    const std::unordered_map<std::string, Series::Description> & series_map,
    std::unordered_map<std::string, Series> & valid_event_series,
    std::unordered_map<std::string, Series> & valid_dag_series,
    const std::unordered_map<std::string, Event> & temp_events = {},
    const std::unordered_map<std::string, DAG> & temp_dags = {}) const;

  /// Validate update event operation
  void validate_update_series_map(
    const std::unordered_map<std::string, Series::Description> & series_map,
    std::unordered_map<std::string, Series> & valid_event_series_map,
    std::unordered_map<std::string, Series> & valid_dag_series_map,
    const std::unordered_map<std::string, Event> & temp_events = {},
    const std::unordered_map<std::string, DAG> & temp_dags = {}) const;

  bool validate_series(
    const Series::Description & series_description,
    Series & valid_series,
    const std::unordered_map<std::string, Event> & temp_events = {},
    const std::unordered_map<std::string, DAG> & temp_dags = {}) const;

  /// Validate delete event operation
  void validate_delete_series_map(
    const std::vector<std::string> & series_ids,
    std::vector<std::string> & valid_event_series_ids,
    std::vector<std::string> & valid_dag_series_ids) const;

  const EventsHandler & events_handler_const() const
  {
    return eh_;
  }

  const std::unordered_map<std::string, DAG> & dags_const() const
  {
    return dags_;
  }

  const std::unordered_map<std::string, Series> & series_map_const() const
  {
    return series_map_;
  }

  EventsHandler & events_handler()
  {
    return eh_;
  }

  std::unordered_map<std::string, DAG> & dags()
  {
    return dags_;
  }

  std::unordered_map<std::string, Series> & series_map()
  {
    return series_map_;
  }

private:
  EventsHandler eh_;
  std::unordered_map<std::string, DAG> dags_;
  std::unordered_map<std::string, Series> series_map_;
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__SCHEDULE_HPP_
