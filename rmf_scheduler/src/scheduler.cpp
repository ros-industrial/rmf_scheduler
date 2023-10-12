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

#include "pluginlib/class_loader.hpp"

#include "rmf_scheduler/scheduler.hpp"
#include "rmf_scheduler/schema_validator.hpp"
#include "rmf_scheduler/error_code.hpp"
#include "rmf_scheduler/utils/uuid.hpp"
#include "rmf_scheduler/utils/system_time_utils.hpp"
#include "rmf_scheduler/parser.hpp"
#include "rmf_scheduler/conflict/identifier.hpp"
#include "rmf_scheduler/conflict/cp_solver.hpp"
#include "rmf_scheduler/log.hpp"

#include "rmf_scheduler/schemas/schedule.hpp"
#include "rmf_scheduler/schemas/event.hpp"
#include "rmf_scheduler/schemas/dependency.hpp"
#include "rmf_scheduler/schemas/series.hpp"
#include "rmf_scheduler/schemas/get_schedule_request.hpp"
#include "rmf_scheduler/schemas/delete_schedule_request.hpp"
#include "rmf_scheduler/schemas/update_event_time.hpp"
#include "rmf_scheduler/schemas/update_series.hpp"
#include "rmf_scheduler/schemas/update_series_request.hpp"

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
    schemas::update_event_time,
    schemas::update_series,
    schemas::update_series_request
  };

  schema_validator_ = std::make_unique<SchemaValidator>(schemas);

  last_write_time_ = utils::now();

  series_expand_time_ = utils::now();

  // Runtime parameters
  spinning_ = false;
}

Scheduler::Scheduler(const SchedulerOptions & options)
: Scheduler()
{
  options_ = options;
}

Scheduler::~Scheduler()
{
}

ErrorCode Scheduler::handle_add_schedule(
  const nlohmann::json & json)
{
  data::Schedule::Description description;
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
    RS_LOG_ERROR(e.what());
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_SCHEMA,
      e.what()
    };
  }

  // Call add_schedule to check logic
  return RMF_SCHEDULER_RESOLVE_ERROR_CODE(add_schedule(description));
}

void Scheduler::add_schedule(const data::Schedule::Description & schedule)
{
  std::unordered_map<std::string, data::Event> events_to_add;
  std::unordered_map<std::string, data::DAG> dags_to_add;
  std::unordered_map<std::string, data::Series> event_series_map_to_add, dag_series_map_to_add;
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

  { // Write lock
    WriteLock lk(mtx_);

    // If there is writing after the read, exit immediately
    if (last_write_time_ > read_time) {
      throw exception::ScheduleMultipleWriteException(
              "Schedule already written by %s at time \n\t%s.",
              last_writer_.c_str(), utils::to_localtime(last_write_time_));
    }

    // Generate task details
    RS_LOG_INFO("Generating task info");
    for (auto & event_itr : events_to_add) {
      task_builder_.build_task(event_itr.second);
    }

    // Trigger estimation pipeline
    RS_LOG_INFO("Starting to estimate duration");
    task_estimator_.estimate(
      events_to_add,
      2.0);


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

    if (options_.expand_series_) {
      schedule_.expand_all_series_until(series_expand_time_);
    }

    // Update who made changes last
    last_write_time_ = utils::now();
    last_writer_ = _thread_id();
  }  // Write Lock

  // tick once since new events are added
  if (spinning_) {
    _tick();
  }
}

ErrorCode Scheduler::handle_update_schedule(
  const nlohmann::json & schedule_json)
{
  data::Schedule::Description description;
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
    RS_LOG_ERROR(e.what());
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_SCHEMA,
      e.what()
    };
  }
  return RMF_SCHEDULER_RESOLVE_ERROR_CODE(update_schedule(description));
}

void Scheduler::update_schedule(const data::Schedule::Description & schedule)
{
  std::unordered_map<std::string, data::Event> events_to_update;
  std::unordered_map<std::string, data::DAG> dags_to_update;
  std::unordered_map<std::string, data::Series>
  event_series_map_to_update, dag_series_map_to_update;
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

  { // Write lock
    WriteLock lk(mtx_);
    // If there is writing after the read, exit immediately
    if (last_write_time_ > read_time) {
      throw exception::ScheduleMultipleWriteException(
              "Schedule already written by %s at time \n\t%s.",
              last_writer_.c_str(), utils::to_localtime(last_write_time_));
    }

    // Generate task details
    RS_LOG_INFO("Generating task info");
    for (auto & event_itr : events_to_update) {
      task_builder_.build_task(event_itr.second);
    }

    // Trigger estimation pipeline
    RS_LOG_INFO("Starting to estimate duration");
    task_estimator_.estimate(events_to_update, 2.0);

    // Update schedule cache
    // Update events first
    for (auto & event_itr : events_to_update) {
      schedule_.update_event(event_itr.second);
    }

    // Update the dependencies
    for (auto & dag_itr : dags_to_update) {
      schedule_.update_dag(dag_itr.first, std::move(dag_itr.second));
    }

    // Update series map
    for (auto & series_itr : event_series_map_to_update) {
      RS_LOG_INFO(
        "Updating series event for series: %s to cron: [%s]",
        series_itr.first.c_str(),
        series_itr.second.cron().c_str());
      schedule_.update_event_series(series_itr.first, std::move(series_itr.second));
    }

    for (auto & series_itr : dag_series_map_to_update) {
      schedule_.update_dag_series(series_itr.first, std::move(series_itr.second));
    }

    if (options_.expand_series_) {
      schedule_.expand_all_series_until(series_expand_time_);
    }

    // Update who made changes last
    last_write_time_ = utils::now();
    last_writer_ = _thread_id();
  }

  // tick once since new events are added
  if (spinning_) {
    _tick();
  }
}

ErrorCode Scheduler::handle_update_series(
  const nlohmann::json & json)
{
  std::unordered_map<std::string, data::Series::Update> update_series_map;
  try {
    auto json_uri = nlohmann::json_uri{
      schemas::update_series_request["$id"]
    };

    // Validate JSON
    schema_validator_->validate(
      json_uri,
      json);

    // Parse the validated json into description
    parser::json_to_update_series_map(json, update_series_map);
  } catch (const std::exception & e) {
    // Invalid Json schema
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_SCHEMA,
      e.what()
    };
  }

  // Call add_schedule to check logic
  return RMF_SCHEDULER_RESOLVE_ERROR_CODE(update_series(update_series_map));
}

void Scheduler::update_series(
  const std::unordered_map<std::string,
  data::Series::Update> & series_updates)
{
  std::unordered_map<std::string, data::Series::Update> event_series_map_to_update,
    dag_series_map_to_update;
  uint64_t read_time;

  {  // Read lock
    ReadLock lk(mtx_);
    read_time = utils::now();

    // Validate the Series
    schedule_.validate_update_full_series_map(
      series_updates,
      event_series_map_to_update,
      dag_series_map_to_update);
  }  // Read Lock

  // Return early if there is nothing to update
  if (event_series_map_to_update.empty() && dag_series_map_to_update.empty()) {
    return;
  }

  { // Write lock
    WriteLock lk(mtx_);
    // If there is writing after the read, exit immediately
    if (last_write_time_ > read_time) {
      throw exception::ScheduleMultipleWriteException(
              "Schedule already written by %s at time \n\t%s.",
              last_writer_.c_str(), utils::to_localtime(last_write_time_));
    }

    // Update series map
    for (auto & series_itr : event_series_map_to_update) {
      RS_LOG_INFO(
        "Updating series event for series: %s to cron: [%s]",
        series_itr.first.c_str(),
        series_itr.second.cron.c_str());

      schedule_.update_event_series_cron(
        series_itr.first,
        series_itr.second.cron,
        series_itr.second.timezone,
        series_itr.second.old_occurrence_time,
        series_itr.second.new_occurrence_time);
    }

    for (auto & series_itr : dag_series_map_to_update) {
      RS_LOG_INFO(
        "Updating series event for series: %s to cron: [%s]",
        series_itr.first.c_str(),
        series_itr.second.cron.c_str());

      schedule_.update_dag_series_cron(
        series_itr.first,
        series_itr.second.cron,
        series_itr.second.timezone,
        series_itr.second.old_occurrence_time,
        series_itr.second.new_occurrence_time);
    }

    schedule_.expand_all_series_until(series_expand_time_);

    // Update who made changes last
    last_write_time_ = utils::now();
    last_writer_ = _thread_id();
  }

  // tick once since new events are added
  if (spinning_) {
    _tick();
  }
}

ErrorCode Scheduler::handle_update_event_time(
  const nlohmann::json & schedule_json)
{
  data::Event update;
  try {
    auto json_uri = nlohmann::json_uri{
      schemas::update_event_time["$id"]
    };

    // Validate JSON
    schema_validator_->validate(
      json_uri,
      schedule_json);

    // Parse the validated json into description
    parser::json_to_update_event_time(schedule_json, update);
  } catch (const std::exception & e) {
    // Invalid Json schema
    RS_LOG_ERROR(e.what());
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_SCHEMA,
      e.what()
    };
  }

  // Translate to schedule update
  data::Schedule::Description description;
  data::Event initial_event;
  try {
    ReadLock lk(mtx_);
    initial_event = schedule_.get_event(update.id);
  } catch (const std::exception & e) {
    RS_LOG_ERROR(e.what());
    return {
      ErrorCode::FAILURE |
      ErrorCode::INVALID_ID |
      ErrorCode::INVALID_EVENT,
      e.what()
    };
  }
  initial_event.start_time = update.start_time;
  initial_event.duration = update.duration;
  description.events[initial_event.id] = initial_event;

  return RMF_SCHEDULER_RESOLVE_ERROR_CODE(update_schedule(description));
}

nlohmann::json Scheduler::handle_get_schedule(
  const nlohmann::json & request_json)
{
  nlohmann::json response_json;
  uint64_t start_time, end_time;
  bool full;
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

    full = request_json.contains("full") ? request_json["full"].get<bool>() : false;
  } catch (const std::exception & e) {
    // Return schema errors
    RS_LOG_ERROR(e.what());
    response_json["error_code"] =
      nlohmann::json {
      {"value", ErrorCode::FAILURE | ErrorCode::INVALID_SCHEMA},
      {"detail", e.what()}
    };
    return response_json;
  }

  try {
    // Check if series expansion is required
    uint64_t max_series_expansion_time = utils::now() +
      static_cast<uint64_t>(options_.series_max_expandable_duration_ * 1e9);

    if (end_time > series_expand_time_ && max_series_expansion_time >
      series_expand_time_ && options_.expand_series_)
    {
      WriteLock w_lk(mtx_);
      schedule_.expand_all_series_until(std::min(end_time, max_series_expansion_time));
    }

    auto schedule_description = get_schedule(start_time, end_time);
    parser::schedule_to_json(schedule_description, response_json["schedule"], full);
    response_json["error_code"] =
      nlohmann::json {
      {"value", ErrorCode::SUCCESS},
      {"detail", ""}
    };
  } catch (const std::exception & e) {
    // Unknown error
    RS_LOG_ERROR(e.what());
    response_json["error_code"] =
      nlohmann::json {
      {"value", ErrorCode::FAILURE},
      {"detail", e.what()}
    };
    return response_json;
  }
  return response_json;
}

data::Schedule::Description Scheduler::get_schedule(
  uint64_t start_time,
  uint64_t end_time) const
{
  ReadLock lk(mtx_);

  // compile the schedule description
  data::Schedule::Description schedule_description;
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


ErrorCode Scheduler::handle_delete_schedule(const nlohmann::json & request_json)
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
    RS_LOG_ERROR(e.what());
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
    throw exception::ScheduleMultipleWriteException(
            "Schedule already written by %s at time \n\t%s.",
            last_writer_.c_str(), utils::to_localtime(last_write_time_));
  }

  // Update schedule cache
  // Delete events from the cache
  // Store dags deleted along the way
  std::unordered_set<std::string> dag_deleted;
  for (auto & event_id : event_ids) {
    data::Event event = schedule_.get_event(event_id);

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
          data::DAG new_dag(schedule_.get_dag(event.dag_id).description());
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

const data::Schedule & Scheduler::schedule_handler_const() const
{
  return schedule_;
}

data::Schedule & Scheduler::schedule_handler()
{
  return schedule_;
}

void Scheduler::optimize(uint64_t start_time, uint64_t end_time)
{
  RS_LOG_INFO(
    "Checking for conflict:\n"
    "\tstart_time: %lus, end time: %lus",
    start_time, end_time);
  auto events = schedule_.events_handler_const().lookup_events(start_time, end_time);
  auto initial_conflicts = utils::identify_conflicts(
    events, {}, {"request::robot", "zone"});
  if (initial_conflicts.empty()) {
    RS_LOG_INFO("No conflict");
    return;
  }

  RS_LOG_INFO("Conflict detected start deconfliction");
  // Initialize the solver
  auto cp_solver = conflict::CpSolver::make();
  cp_solver->init(events, {start_time, end_time}, 5.0);

  // Categorise the events by type
  auto event_by_type = utils::categorise_by_type(events);

  // Mark flight schedule as fixed
  auto flight_event_itr = event_by_type.find("flight-schedule");
  if (flight_event_itr != event_by_type.end()) {
    cp_solver->mark_fixed(flight_event_itr->second);
  }

  // Add robot task to the objective function
  auto robot_event_itr = event_by_type.find("Cleaning Task");
  if (robot_event_itr != event_by_type.end()) {
    cp_solver->add_objective(robot_event_itr->second);
  }

  // Solve for a feasible solution
  auto event_by_filter = utils::categorise_by_filter(events, {"robot", "zone"});

  for (auto itr : event_by_filter) {
    for (auto itr2 : itr) {
      cp_solver->add_no_overlap(itr2.second);
    }
  }

  // Solve for a way to avoid conflict
  auto result = cp_solver->solve(true);

  RS_LOG_INFO("Solver done");
  // Implement the changes
  for (auto & change : result) {
    RS_LOG_INFO(
      "Event id: %s\n"
      "orig: %lus, final:%lus.",
      change.id.c_str(),
      change.original_start_time,
      change.final_start_time);
    auto event = schedule_.get_event(change.id);
    event.start_time = change.final_start_time;
    schedule_.update_event(event);
  }
}

void Scheduler::load_runtime_interface(
  const std::shared_ptr<void> & node,
  const std::string & name,
  const std::string & interface,
  const std::vector<std::string> & supported_task_types)
{
  task_executor_.load_plugin(node, name, interface, supported_task_types);
}

void Scheduler::unload_runtime_interface(
  const std::string & name)
{
  task_executor_.unload_plugin(name);
}

void Scheduler::load_estimate_interface(
  const std::shared_ptr<void> & node,
  const std::string & name,
  const std::string & interface,
  const std::vector<std::string> & supported_task_types)
{
  task_estimator_.load_plugin(node, name, interface, supported_task_types);
}

void Scheduler::unload_estimate_interface(
  const std::string & name)
{
  task_estimator_.unload_plugin(name);
}

void Scheduler::load_builder_interface(
  const std::shared_ptr<void> & node,
  const std::string & name,
  const std::string & interface,
  const std::vector<std::string> & supported_task_types)
{
  task_builder_.load_plugin(node, name, interface, supported_task_types);
}

void Scheduler::unload_builder_interface(
  const std::string & name)
{
  task_builder_.unload_plugin(name);
}

std::string Scheduler::_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

void Scheduler::spin()
{
  // this funtion can only be called once
  if (spinning_) {
    throw exception::TaskExecutionException("Already spinnning");
  }
  spinning_ = true;
  uint64_t tick_time =
    utils::now();

  // Add in the first tick event
  std::string ticking_event_id = "tick-" + utils::gen_uuid();
  data::Event ticking_event {
    "Ticking Event",
    "ticking_event",               // type
    tick_time,
    0,                          // duration
    ticking_event_id,           // id
    "",                         // series id
    "",                         // dag id
    ""                          // event details
  };

  // Keep track of the ticking time
  next_tick_time_ = tick_time;

  // Minimal tick rate
  tick_period_ = options_.tick_period_;

  ste_.add_action(
    ticking_event,
    [this]() -> void {
      _tick();
    });

  RS_LOG_INFO("Spinning..");
  ste_.spin();
}

void Scheduler::stop()
{
  ste_.stop();
}

void Scheduler::_tick()
{
  // Write lock
  WriteLock lk(mtx_);  // TODO(Briancbn): Improve mutex locks

  // Add in next tick
  // First modulate the ticking period
  uint64_t current_time = utils::now();
  RS_LOG_INFO("Ticking Current Time: %lus", current_time);

  // Only tick when past the next tick time
  if (next_tick_time_ <= current_time) {
    next_tick_time_ = next_tick_time_ + static_cast<uint64_t>(tick_period_) * 1e9;

    RS_LOG_INFO("Add next timer tick at time %lus", next_tick_time_);

    // Add in next tick event
    std::string ticking_event_id = "tick-" + utils::gen_uuid();
    data::Event ticking_event {
      "Ticking Event",
      "ticking_event",            // type
      next_tick_time_,            // next tick time
      0,                          // duration
      ticking_event_id,           // id
      "",                         // series id
      "",                         // dag id
      ""                          // event details
    };

    ste_.add_action(
      ticking_event,
      [this]() -> void {
        _tick();
      });
  }

  // push events
  uint64_t tick_period_start_time = current_time -
    static_cast<uint64_t>(options_.allow_past_events_duration_) * 1e9;
  uint64_t tick_period_end_time = next_tick_time_;

  // Push these events to the queue
  _push_events(tick_period_start_time, tick_period_end_time);

  // TODO(Briancbn): free up unused resources
}

void Scheduler::_push_events(uint64_t start_time, uint64_t end_time)
{
  // First expand all series till end_time
  if (end_time > series_expand_time_) {
    schedule_.expand_all_series_until(end_time);
  }

  // Retrieve all events within the window
  auto events = schedule_.events_handler_const().lookup_events(start_time, end_time);

  RS_LOG_INFO("Found %lu events to push to runtime", events.size());
  for (auto & event : events) {
    // Filter event that cannot be executed
    if (!task_executor_.is_supported(event.type)) {
      RS_LOG_INFO(
        "Event type [%s] doesn't have a runtime interface, skipping..",
        event.type.c_str());
      continue;
    }

    bool dag_action = !event.dag_id.empty();
    std::string action_id = dag_action ? event.dag_id : event.id;

    // Check ignore actions that are already pushed
    if (pushed_action_ids_.find(action_id) != pushed_action_ids_.end()) {
      continue;
    }

    RS_LOG_INFO(
      "Pushing %s action [%s] to runtime",
      (dag_action ? "DAG" : "Event"),
      action_id.c_str());

    // Generate action
    runtime::SystemTimeExecutor::Action action;
    if (dag_action) {
      // Create a fake event specifically for DAG
      data::DAG & dag = schedule_.dags()[action_id];
      // Get dag start time
      uint64_t dag_start_time = schedule_.get_dag_start_time(dag);
      data::Event dag_event {
        "DAG Event",
        "dag_event",                // type
        dag_start_time,             // next tick time
        0,                          // duration
        action_id,                  // id
        "",                         // series id
        "",                         // dag id
        ""                          // event details
      };
      action = _generate_dag_action(dag);
      ste_.add_action(dag_event, action);
    } else {
      action = _generate_event_action(event);
      ste_.add_action(event, action);
    }
    pushed_action_ids_.emplace(action_id);
  }
}

runtime::SystemTimeExecutor::Action Scheduler::_generate_dag_action(
  data::DAG & dag)
{
  return [ =, &dag]() -> void {
           // Write lock
           WriteLock lk(mtx_);
           runtime::DAGExecutor dag_executor;
           auto future = dag_executor.run(
             dag,
             [this](const std::string & id) -> runtime::DAGExecutor::Work {
               data::Event event = schedule_.get_event(id);
               return _generate_event_action(event, true);
             });
           dag_executors_.emplace_back(std::move(dag_executor));
           dag_futures_.push_back(future);
         };
}

runtime::SystemTimeExecutor::Action Scheduler::_generate_event_action(
  const data::Event & event,
  bool blocking)
{
  return [ = ]() {
           auto future = task_executor_.run_async(event);
           if (blocking) {
             future.wait();
           }
         };
}

}  // namespace rmf_scheduler
