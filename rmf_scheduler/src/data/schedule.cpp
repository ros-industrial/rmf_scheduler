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

#include <ostream>

#include "rmf_scheduler/data/schedule.hpp"
#include "rmf_scheduler/runtime/dag_executor.hpp"

#include "rmf_scheduler/log.hpp"
#include "nlohmann/json.hpp"

namespace rmf_scheduler
{

namespace data
{

namespace internal
{

class EventsHandlerSeriesExtension : public Series::ObserverBase
{
public:
  EventsHandlerSeriesExtension(
    EventsHandler & eh)
  : eh_(eh)
  {
  }

  void on_add_occurrence(
    const Series::Occurrence & ref_occurrence,
    const Series::Occurrence & new_occurrence) override
  {
    auto event = eh_.get_event(ref_occurrence.id);
    event.id = new_occurrence.id;
    event.start_time = new_occurrence.time;
    eh_.add_event(event);
  }

  void on_update_occurrence(
    const Series::Occurrence & old_occurrence,
    uint64_t new_time) override
  {
    auto event = eh_.get_event(old_occurrence.id);
    event.start_time = new_time;
    eh_.update_event(event);
  }

  void on_update_cron(
    const Series::Occurrence & /*old_occurrence*/,
    uint64_t /*new_time*/) override
  {
  }

  void on_delete_occurrence(
    const Series::Occurrence & occurrence) override
  {
    eh_.delete_event(occurrence.id);
  }

private:
  EventsHandler & eh_;
};

class DAGSeriesExtension : public Series::ObserverBase
{
public:
  DAGSeriesExtension(
    EventsHandler & eh,
    std::unordered_map<std::string, DAG> & dags)
  : eh_(eh), dags_(dags)
  {
  }

  void on_add_occurrence(
    const Series::Occurrence & ref_occurrence,
    const Series::Occurrence & new_occurrence) override
  {
    auto ref_description = dags_[ref_occurrence.id].description();
    DAG new_dag = DAG(ref_description);
    auto all_nodes = new_dag.all_nodes();
    for (auto & node_id : all_nodes) {
      std::string new_node_id = node_id;
      size_t found = new_node_id.find(ref_occurrence.id);
      // Replace exiting occurence ID if possible
      if (found != std::string::npos) {
        new_node_id.replace(found, ref_occurrence.id.size(), new_occurrence.id);
      } else {
        new_node_id = new_occurrence.id + "-" + node_id;
      }
      new_dag.update_node(node_id, new_node_id);

      // Add new events
      Event event = eh_.get_event(node_id);
      event.id = new_node_id;
      event.start_time = event.start_time - ref_occurrence.time + new_occurrence.time;
      event.dag_id = new_occurrence.id;
      eh_.add_event(event);
    }
    dags_[new_occurrence.id] = std::move(new_dag);
  }

  void on_update_occurrence(
    const Series::Occurrence & /*old_occurrence*/,
    uint64_t /*new_time*/) override
  {
    // Do nothing;
  }

  void on_update_cron(
    const Series::Occurrence & old_occurrence,
    uint64_t new_time) override
  {
    // Update starting time for all events within the dag occurrence
    auto all_nodes = dags_.at(old_occurrence.id).all_nodes();
    for (auto & node_id : all_nodes) {
      Event event = eh_.get_event(node_id);
      event.start_time = event.start_time - old_occurrence.time + new_time;
      eh_.update_event(event);
    }
  }

  void on_delete_occurrence(
    const Series::Occurrence & occurrence) override
  {
    auto dag_itr = dags_.find(occurrence.id);
    auto all_nodes = dag_itr->second.all_nodes();
    for (auto & node_id : all_nodes) {
      eh_.delete_event(node_id);
    }
    dags_.erase(dag_itr);
  }

private:
  // TODO(Briancbn): Use weak_ptr instead
  EventsHandler & eh_;
  std::unordered_map<std::string, DAG> & dags_;
};
}  // namespace internal

void Schedule::add_event(const Event & event)
{
  eh_.add_event(event);
}

void Schedule::update_event(const Event & event)
{
  // Update the series occurrence only if it is an event serie
  if (!event.series_id.empty() && event.dag_id.empty()) {
    uint64_t old_start_time = eh_.get_event(event.id).start_time;
    auto series_itr = series_map_.find(event.series_id);
    if (series_itr == series_map_.end()) {
      throw exception::SeriesIDException(
              event.series_id.c_str(),
              "Event [%s] with Series ID [%s] does not exist in schedule",
              event.id.c_str(),
              event.series_id.c_str());
    }
    series_itr->second.update_occurrence_time(old_start_time, event.start_time);
  }
  eh_.update_event(event);
}

void Schedule::delete_event(const std::string & event_id)
{
  eh_.delete_event(event_id);
}

Event Schedule::get_event(const std::string & event_id) const
{
  return eh_.get_event(event_id);
}

void Schedule::add_dag(
  const std::string & dag_id,
  DAG && dag)
{
  for (auto & event_id : dag.all_nodes()) {
    Event event = eh_.get_event(event_id);
    event.dag_id = dag_id;
    eh_.update_event(event);
  }

  dags_[dag_id] = std::move(dag);
}

void Schedule::add_dag_dependency(
  const std::string & dag_id,
  const std::string & event_id,
  const DAG::DependencyInfo & dependency_info)
{
  for (auto & dependant_event_id : dependency_info) {
    Event event = eh_.get_event(dependant_event_id);
    event.dag_id = dag_id;
    eh_.update_event(event);
  }
  dags_[dag_id].add_dependency(event_id, dependency_info);
}

void Schedule::update_dag(
  const std::string & dag_id,
  DAG && dag)
{
  std::string series_id;
  bool series_found = false;
  for (auto & event_id : dags_.at(dag_id).all_nodes()) {
    Event event = eh_.get_event(event_id);
    if (!event.series_id.empty() && !series_found) {
      series_id = event.series_id;
      series_found = true;
    }
    event.dag_id = "";
    event.series_id = "";
    eh_.update_event(event);
  }
  for (auto & event_id : dag.all_nodes()) {
    Event event = eh_.get_event(event_id);
    event.dag_id = dag_id;
    if (event.series_id.empty() && series_found) {
      event.series_id = series_id;
    }
    eh_.update_event(event);
  }
  dags_[dag_id] = std::move(dag);

  // Update dag start time here so we can use it later
  generate_dag_event_start_time(dag_id);

  // Generate the start time for events based on DAG
  if (series_found) {
    auto series_itr = series_map_.find(series_id);
    if (series_itr == series_map_.end()) {
      throw exception::SeriesIDException(
              series_id.c_str(),
              "Series ID [%s] from DAG [%s] does not exist in schedule",
              series_id.c_str(),
              dag_id.c_str());
    }
    uint64_t old_dag_start_time = series_itr->second.get_occurrence_time(dag_id);

    // Delete the DAG occurrence if it is empty
    if (dags_.at(dag_id).all_nodes().empty()) {
      series_itr->second.delete_occurrence(old_dag_start_time);
      return;
    }
    // update_dag_series_occurrence(
    uint64_t new_dag_start_time = get_dag_start_time(dag_id);
    std::ostringstream oss;
    oss << "Old start time ["
        << old_dag_start_time
        << "].\n New start time ["
        << new_dag_start_time
        << "]";
    RS_LOG_DEBUG(oss.str().c_str());

    series_itr->second.update_occurrence_time(old_dag_start_time, new_dag_start_time);
  }
}

void Schedule::detach_dag_event(
  const std::string & dag_id,
  const std::string & event_id)
{
  Event event = eh_.get_event(event_id);
  event.dag_id = "";
  dags_[dag_id].delete_node(event_id);
}

void Schedule::delete_dag(
  const std::string & dag_id)
{
  auto dag_itr = dags_.find(dag_id);
  auto events_to_delete = dag_itr->second.all_nodes();
  for (auto & event_id : events_to_delete) {
    eh_.delete_event(event_id);
  }
  dags_.erase(dag_itr);
}

void Schedule::generate_dag_event_start_time(const std::string & dag_id)
{
  // Perform a fake execution to estimate time
  // Contains all of the calculated start time from execution of the task ahead
  std::unordered_map<std::string, uint64_t> calculated_end_time;

  // Use single thread for simplification
  runtime::DAGExecutor dag_executor(1);
  auto & dag = dags_[dag_id];
  runtime::DAGExecutor::WorkGenerator work_generator =
    [&calculated_end_time,
      & eh = eh_, &dag](const std::string & id)
    -> runtime::DAGExecutor::Work {
      // Check if the start time is calculated
      auto dep_info = dag.get_dependency_info(id);

      // Generate an task to update end time based on dependency
      // and update the end time
      return [ =, &calculated_end_time, &eh]() {
               Event event = eh.get_event(id);
               if (!dep_info.empty()) {
                 // Find the latest end time of all the dependent task as the new start time
                 uint64_t new_start_time = 0;
                 for (auto & dep : dep_info) {
                   uint64_t dep_end_time = calculated_end_time.at(dep);
                   if (dep_end_time > new_start_time) {
                     new_start_time = dep_end_time;
                   }
                 }
                 event.start_time = new_start_time;
                 eh.update_event(event);
               }
               calculated_end_time[id] = event.start_time + event.duration;
             };
    };
  // Run the execution
  auto future = dag_executor.run(dag.description(), work_generator);
  future.get();
}

const DAG & Schedule::get_dag(const std::string & dag_id) const
{
  return dags_.at(dag_id);
}

uint64_t Schedule::get_dag_start_time(
  const DAG & dag,
  const std::unordered_map<std::string, Event> & temp_events) const
{
  uint64_t start_time = UINT64_MAX;
  for (auto node_id : dag.entry_nodes()) {
    uint64_t entry_start_time;
    auto itr = temp_events.find(node_id);
    if (itr != temp_events.end()) {
      entry_start_time = itr->second.start_time;
    } else {
      entry_start_time = eh_.get_event(node_id).start_time;
    }

    // Use the earliest start time
    if (entry_start_time < start_time) {
      start_time = entry_start_time;
    }
  }
  return start_time;
}

uint64_t Schedule::get_dag_start_time(const std::string & dag_id) const
{
  auto & dag = dags_.at(dag_id);
  // Send an empty map of events so that we will use existing events
  return get_dag_start_time(dag);
}

void Schedule::add_event_series(
  const std::string & series_id,
  Series && series)
{
  auto occurrences = series.occurrences();
  for (auto & occurrence : occurrences) {
    Event event = eh_.get_event(occurrence.id);
    event.series_id = series_id;
    eh_.update_event(event);
  }

  series_map_[series_id] = std::move(series);

  // Create series observer
  series_map_[series_id].make_observer<internal::EventsHandlerSeriesExtension>(eh_);
}

void Schedule::update_event_series(
  const std::string & series_id,
  Series && series)
{
  auto old_occurrences = series_map_.at(series_id).occurrences();
  for (auto & occurrence : old_occurrences) {
    Event event = eh_.get_event(occurrence.id);
    event.series_id = "";
    eh_.update_event(event);
  }

  auto occurrences = series.occurrences();
  for (auto & occurrence : occurrences) {
    Event event = eh_.get_event(occurrence.id);
    event.series_id = series_id;
    eh_.update_event(event);
  }

  series_map_[series_id] = std::move(series);

  // Create series observer
  series_map_[series_id].make_observer<internal::EventsHandlerSeriesExtension>(eh_);
}

void Schedule::expand_event_series_until(
  const std::string & series_id,
  uint64_t time)
{
  series_map_[series_id].expand_until(time);
}

void Schedule::update_event_series_cron(
  const std::string & series_id,
  const std::string & new_cron,
  const std::string & timezone,
  uint64_t old_occurrence_time,
  uint64_t new_occurrence_time)
{
  auto & series = series_map_[series_id];
  series.update_cron_from(old_occurrence_time, new_occurrence_time, new_cron, timezone);
}

void Schedule::update_event_series_cron(
  const std::string & series_id,
  const Event & new_event,
  const std::string & new_cron,
  const std::string & timezone)
{
  auto old_event = eh_.get_event(new_event.id);
  auto & series = series_map_[series_id];
  series.update_cron_from(old_event.start_time, new_event.start_time, new_cron, timezone);
}

void Schedule::update_dag_series_cron(
  const std::string & series_id,
  const std::string & new_cron,
  const std::string & timezone,
  uint64_t old_occurrence_time,
  uint64_t new_occurrence_time)
{
  auto & series = series_map_[series_id];
  series.update_cron_from(old_occurrence_time, new_occurrence_time, new_cron, timezone);
}

void Schedule::update_event_series_until(
  const std::string & series_id,
  uint64_t time)
{
  series_map_[series_id].set_until(time);
}

void Schedule::update_event_series_occurrence(
  const std::string & series_id,
  const Event & new_event)
{
  auto old_event = eh_.get_event(new_event.id);
  series_map_[series_id].update_occurrence_time(old_event.start_time, new_event.start_time);
}

void Schedule::delete_event_series_occurrence(
  const std::string & series_id,
  const std::string & event_id)
{
  auto old_event = eh_.get_event(event_id);
  series_map_[series_id].delete_occurrence(old_event.start_time);
}

void Schedule::detach_event_series_occurrence(
  const std::string & series_id,
  const std::string & event_id)
{
  auto old_event = eh_.get_event(event_id);
  series_map_[series_id].delete_occurrence(old_event.start_time);

  // Add back the old event
  old_event.series_id = "";
  eh_.add_event(old_event);
}

void Schedule::delete_event_series(
  const std::string & series_id)
{
  auto itr = series_map_.find(series_id);
  auto occurrences = itr->second.occurrences();
  for (auto & occurrence : occurrences) {
    eh_.delete_event(occurrence.id);
  }
  series_map_.erase(itr);
}

void Schedule::add_dag_series(
  const std::string & series_id,
  Series && series)
{
  auto occurrences = series.occurrences();
  for (auto & occurrence : occurrences) {
    const DAG & dag = dags_.at(occurrence.id);
    for (auto & node_id : dag.all_nodes()) {
      Event event = eh_.get_event(node_id);
      event.series_id = series_id;
      eh_.update_event(event);
    }
  }

  series_map_[series_id] = std::move(series);

  // Create series observer
  series_map_[series_id].make_observer<internal::DAGSeriesExtension>(eh_, dags_);
}

void Schedule::update_dag_series(
  const std::string & series_id,
  Series && series)
{
  auto old_occurrences = series.occurrences();
  for (auto & occurrence : old_occurrences) {
    const DAG & dag = dags_.at(occurrence.id);
    for (auto & node_id : dag.all_nodes()) {
      Event event = eh_.get_event(node_id);
      event.series_id = "";
      eh_.update_event(event);
    }
  }

  auto occurrences = series.occurrences();
  for (auto & occurrence : occurrences) {
    const DAG & dag = dags_.at(occurrence.id);
    for (auto & node_id : dag.all_nodes()) {
      Event event = eh_.get_event(node_id);
      event.series_id = series_id;
      eh_.update_event(event);
    }
  }

  series_map_[series_id] = std::move(series);

  // Create series observer
  series_map_[series_id].make_observer<internal::DAGSeriesExtension>(eh_, dags_);
}

void Schedule::expand_dag_series_until(
  const std::string & series_id,
  uint64_t time)
{
  series_map_[series_id].expand_until(time);
}

void Schedule::update_dag_series_cron(
  const std::string & series_id,
  const std::string & dag_id,
  const DAG & new_dag,
  const std::string & new_cron,
  const std::string & timezone)
{
  const auto & old_dag = dags_.at(dag_id);
  uint64_t old_start_time = get_dag_start_time(old_dag);
  uint64_t new_start_time = get_dag_start_time(new_dag);
  auto & series = series_map_[series_id];
  series.update_cron_from(old_start_time, new_start_time, new_cron, timezone);
}

void Schedule::update_dag_series_until(
  const std::string & series_id,
  uint64_t time)
{
  series_map_[series_id].set_until(time);
}

void Schedule::update_dag_series_occurrence(
  const std::string & series_id,
  const std::string & dag_id,
  const DAG & new_dag)
{
  const auto & old_dag = dags_.at(dag_id);
  uint64_t old_start_time = get_dag_start_time(old_dag);
  uint64_t new_start_time = get_dag_start_time(new_dag);

  auto & series = series_map_[series_id];
  series.update_occurrence_time(old_start_time, new_start_time);
}

void Schedule::delete_dag_series_occurrence(
  const std::string & series_id,
  const std::string & dag_id)
{
  const auto & dag = dags_.at(dag_id);
  uint64_t start_time = get_dag_start_time(dag);
  series_map_[series_id].delete_occurrence(start_time);
}

void Schedule::detach_dag_series_occurrence(
  const std::string & series_id,
  const std::string & dag_id)
{
  const auto & dag = dags_.at(dag_id);
  auto dag_description = dag.description();
  auto all_nodes = dag.all_nodes();
  // Get all old events
  std::vector<Event> old_events;
  for (auto & node : all_nodes) {
    old_events.push_back(eh_.get_event(node));
  }

  uint64_t start_time = get_dag_start_time(dag);
  series_map_[series_id].delete_occurrence(start_time);

  // Add back old event and dag
  for (auto & event : old_events) {
    event.series_id = "";
    eh_.add_event(event);
  }
  dags_.emplace(dag_id, dag_description);
}

void Schedule::delete_dag_series(const std::string & series_id)
{
  auto itr = series_map_.find(series_id);

  auto occurrences = itr->second.occurrences();
  for (auto & occurrence : occurrences) {
    delete_dag(occurrence.id);
  }
  series_map_.erase(itr);
}

void Schedule::expand_all_series_until(uint64_t time)
{
  for (auto & series : series_map_) {
    series.second.expand_until(time);
  }
}

const Series & Schedule::get_series(
  const std::string & series_id) const
{
  return series_map_.at(series_id);
}

void Schedule::purge()
{
  for (auto series_itr = series_map_.begin();
    series_itr != series_map_.end(); )
  {
    if (series_itr->second.occurrences().empty()) {
      series_itr = series_map_.erase(series_itr);
    } else {
      series_itr++;
    }
  }

  for (auto dag_itr = dags_.begin();
    dag_itr != dags_.end(); )
  {
    if (dag_itr->second.all_nodes().empty()) {
      dag_itr = dags_.erase(dag_itr);
    } else {
      dag_itr++;
    }
  }
}

void Schedule::validate_add_events(
  const std::unordered_map<std::string, Event> & events,
  std::unordered_map<std::string, Event> & valid_events) const
{
  for (auto & event_itr : events) {
    // throw error if the events already exists
    if (eh_.has_event(event_itr.first)) {
      throw exception::EventsHandlerIDException(
              event_itr.first.c_str(),
              "add_event error "
              "Event [%s] already exists, use a different ID",
              event_itr.first.c_str());
    }
    // Keep series_id and dag_ids empty
    auto result = valid_events.emplace(event_itr.first, event_itr.second);
    result.first->second.dag_id = "";
    result.first->second.series_id = "";
  }
}

void Schedule::validate_update_events(
  const std::unordered_map<std::string, Event> & events,
  std::unordered_map<std::string, Event> & valid_events) const
{
  for (auto event_itr : events) {
    // throw error if the events DOESN'T exists
    if (!eh_.has_event(event_itr.first)) {
      throw exception::EventsHandlerIDException(
              event_itr.first.c_str(),
              "update_event error "
              "Event [%s] doesn't exists, use a different ID",
              event_itr.first.c_str());
    }
    // Keep series_id and dag_ids unchanged
    Event old_event = eh_.get_event(event_itr.first);
    auto result = valid_events.emplace(event_itr.first, event_itr.second);
    result.first->second.dag_id = old_event.dag_id;
    result.first->second.series_id = old_event.series_id;
  }
}

void Schedule::validate_delete_events(
  const std::vector<std::string> & event_ids) const
{
  for (auto & event_id : event_ids) {
    if (!eh_.has_event(event_id)) {
      throw exception::EventsHandlerIDException(
              event_id.c_str(),
              "delete_events error "
              "Event [%s] doesn't exists, use a different ID",
              event_id.c_str());
    }
  }
}

void Schedule::validate_add_dags(
  const std::unordered_map<std::string, DAG::Description> & dags,
  std::unordered_map<std::string, DAG> & valid_dags,
  const std::unordered_map<std::string, Event> & temp_events) const
{
  for (auto & dag_itr : dags) {
    // throw error if DAG graph ID overlaps with an existing one
    if (dags_.find(dag_itr.first) != dags_.end()) {
      throw exception::DAGIDException(
              dag_itr.first.c_str(),
              "add_dependency error "
              "DAG graph [%s] already exists, use a different ID",
              dag_itr.first.c_str());
    }

    DAG temp_dag;

    // Validate DAG description
    validate_dag(dag_itr.second, temp_dag, temp_events);
    valid_dags.emplace(dag_itr.first, std::move(temp_dag));
  }
}

void Schedule::validate_update_dags(
  const std::unordered_map<std::string, DAG::Description> & dags,
  std::unordered_map<std::string, DAG> & valid_dags) const
{
  for (auto & dag_itr : dags) {
    // throw error if DAG graph doesn't exist
    if (dags_.find(dag_itr.first) == dags_.end()) {
      throw exception::DAGIDException(
              dag_itr.first.c_str(),
              "update_dependency error "
              "DAG graph [%s] doesn't exist, use a different ID",
              dag_itr.first.c_str());
    }

    DAG temp_dag;

    // Validate DAG description
    validate_dag(dag_itr.second, temp_dag);
    valid_dags.emplace(dag_itr.first, std::move(temp_dag));
  }
}

void Schedule::validate_dag(
  const DAG::Description & dag_description,
  DAG & valid_dag,
  const std::unordered_map<std::string, Event> & temp_events) const
{
  // Check if DAGs are cyclic
  DAG temp_dag = DAG(dag_description, true);  // Throw DAGCyclicException or DAGIDException

  // Validate that All IDs in the dags exist in newly added events or existing events
  for (auto & node_id : temp_dag.all_nodes()) {
    if (eh_.has_event(node_id)) {
      continue;
    }
    auto event_itr = temp_events.find(node_id);
    if (event_itr == temp_events.end()) {
      throw exception::DAGIDException(
              node_id.c_str(),
              "dependency error "
              "Event ID [%s] in dependency graph doesn't exist",
              node_id.c_str());
    }
  }
  valid_dag = std::move(temp_dag);
}

void Schedule::validate_delete_dags(
  const std::vector<std::string> & dag_ids) const
{
  for (auto & dag_id : dag_ids) {
    auto dag_itr = dags_.find(dag_id);
    if (dag_itr == dags_.end()) {
      throw exception::DAGIDException(
              dag_id.c_str(),
              "delete_dependency error "
              "dependency [%s] doesn't exists, use a different ID",
              dag_id.c_str());
    }
  }
}

bool Schedule::is_dag_series(
  const std::vector<Series::Occurrence> & occurrences,
  const std::unordered_map<std::string, Event> & temp_events,
  const std::unordered_map<std::string, DAG> & temp_dags) const
{
  if (occurrences.empty()) {
    throw exception::SeriesEmptyException("Series is empty");
  }

  auto first_occurrence = occurrences.front();
  if (eh_.has_event(first_occurrence.id) ||
    temp_events.find(first_occurrence.id) != temp_events.end())
  {
    return false;
  } else if (dags_.find(first_occurrence.id) != dags_.end() ||  // NOLINT(readability/braces)
    temp_dags.find(first_occurrence.id) != temp_dags.end())
  {
    return true;
  } else {
    throw exception::SeriesIDException(
            first_occurrence.id.c_str(),
            "Occurrence [%s] is neither an event nor a dependency graph",
            first_occurrence.id.c_str());
  }
}

void Schedule::validate_add_series_map(
  const std::unordered_map<std::string, Series::Description> & series_map,
  std::unordered_map<std::string, Series> & valid_event_series_map,
  std::unordered_map<std::string, Series> & valid_dag_series_map,
  const std::unordered_map<std::string, Event> & temp_events,
  const std::unordered_map<std::string, DAG> & temp_dags) const
{
  // Validate all ids in the series exists in the events or DAG
  for (auto & series_itr : series_map) {
    // First check if the Series ID overlaps with an existing one
    if (series_map_.find(series_itr.first) != series_map_.end()) {
      throw exception::SeriesIDException(
              series_itr.first.c_str(),
              "add_series error "
              "Series [%s] already exists, use a different ID",
              series_itr.first.c_str());
    }

    // Valid the individual series
    // This also provide start time for the series description
    Series valid_series;
    bool is_dag_series = validate_series(
      series_itr.second, valid_series, temp_events, temp_dags);

    if (is_dag_series) {
      valid_dag_series_map.emplace(series_itr.first, std::move(valid_series));
    } else {
      valid_event_series_map.emplace(series_itr.first, std::move(valid_series));
    }
  }
}

void Schedule::validate_update_full_series_map(
  const std::unordered_map<std::string, Series::Update> & series_map,
  std::unordered_map<std::string, Series::Update> & valid_update_event_series_map,
  std::unordered_map<std::string, Series::Update> & valid_update_dag_series_map) const
{
  // Validate all ids in the series exists in the events or DAG
  for (auto & series_itr : series_map) {
    // Throw error if the Series ID is not an existing one
    auto old_series_itr = series_map_.find(series_itr.first);
    if (old_series_itr == series_map_.end()) {
      throw exception::SeriesIDException(
              series_itr.first.c_str(),
              "update_series error "
              "Series [%s] doesn't exist, use a different ID",
              series_itr.first.c_str());
    }

    if (is_dag_series(old_series_itr->second.occurrences())) {
      valid_update_dag_series_map.emplace(series_itr.first, series_map.at(series_itr.first));
    } else {
      valid_update_event_series_map.emplace(series_itr.first, series_map.at(series_itr.first));
    }
  }
}

void Schedule::validate_update_series_map(
  const std::unordered_map<std::string, Series::Description> & series_map,
  std::unordered_map<std::string, Series> & valid_event_series_map,
  std::unordered_map<std::string, Series> & valid_dag_series_map,
  const std::unordered_map<std::string, Event> & temp_events,
  const std::unordered_map<std::string, DAG> & temp_dags) const
{
  // Validate all ids in the series exists in the events or DAG
  for (auto & series_itr : series_map) {
    // Throw error if the Series ID is not an existing one
    auto old_series_itr = series_map_.find(series_itr.first);
    if (old_series_itr == series_map_.end()) {
      throw exception::SeriesIDException(
              series_itr.first.c_str(),
              "update_series error "
              "Series [%s] doesn't exist, use a different ID",
              series_itr.first.c_str());
    }
    // Valid the individual series
    // This also provide start time for the series description
    Series valid_series;
    bool is_dag_series = validate_series(
      series_itr.second, valid_series, temp_events, temp_dags);

    // If old series is DAG series, new series should be too. Same for event series.
    if (this->is_dag_series(old_series_itr->second.occurrences()) != is_dag_series) {
      throw exception::SeriesIDException(
              series_itr.first.c_str(),
              "update_series error "
              "Series [%s] doesn't match the type of the old series",
              series_itr.first.c_str());
    }


    if (is_dag_series) {
      valid_dag_series_map.emplace(series_itr.first, std::move(valid_series));
    } else {
      valid_event_series_map.emplace(series_itr.first, std::move(valid_series));
    }
  }
}

bool Schedule::validate_series(
  const Series::Description & series_description,
  Series & valid_series,
  const std::unordered_map<std::string, Event> & temp_events,
  const std::unordered_map<std::string, DAG> & temp_dags) const
{
  // Create a copy to modify the occurrence start time
  auto series_description_w_time = series_description;

  bool is_dag_series = this->is_dag_series(
    series_description.occurrences, temp_events, temp_dags);
  // Check if the occurrences exists
  for (auto & occurrence_itr : series_description_w_time.occurrences) {
    // Add start time to series
    if (is_dag_series) {
      // Series is for DAG
      auto dag_itr = temp_dags.find(occurrence_itr.id);
      if (dag_itr == temp_dags.end()) {
        dag_itr = dags_.find(occurrence_itr.id);
        if (dag_itr == dags_.end()) {
          // Occurrence doesn't exist
          throw exception::SeriesIDException(
                  occurrence_itr.id.c_str(),
                  "Occurrence [%s] in dag series doesn't exist.",
                  occurrence_itr.id.c_str());
        }
      }
      occurrence_itr.time = get_dag_start_time(dag_itr->second, temp_events);
    } else {
      // Series is for event
      // Attach time to the occurrence
      Event event;
      auto event_itr = temp_events.find(occurrence_itr.id);
      if (event_itr != temp_events.end()) {
        event = event_itr->second;
      } else {
        if (!eh_.has_event(occurrence_itr.id)) {
          // Occurrence doesn't exist
          throw exception::SeriesIDException(
                  occurrence_itr.id.c_str(),
                  "Occurrence [%s] in event series doesn't exist.",
                  occurrence_itr.id.c_str());
        }
        event = eh_.get_event(occurrence_itr.id);
      }
      occurrence_itr.time = event.start_time;
    }
  }

  // Validate crons
  // throw SeriesEmptyException or SeriesInvalidCronException
  valid_series = Series(series_description_w_time);
  return is_dag_series;
}

void Schedule::validate_delete_series_map(
  const std::vector<std::string> & series_ids,
  std::vector<std::string> & valid_event_series_ids,
  std::vector<std::string> & valid_dag_series_ids) const
{
  for (auto & series_id : series_ids) {
    auto series_itr = series_map_.find(series_id);
    if (series_itr == series_map_.end()) {
      throw exception::SeriesIDException(
              series_id.c_str(),
              "delete_series error "
              "series [%s] doesn't exists, use a different ID",
              series_id.c_str());
    }
    if (is_dag_series(series_itr->second.occurrences())) {
      valid_dag_series_ids.push_back(series_id);
    } else {
      valid_event_series_ids.push_back(series_id);
    }
  }
}

}  // namespace data

}  // namespace rmf_scheduler
