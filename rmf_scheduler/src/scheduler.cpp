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
#include <regex>

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

std::string to_slug(const std::string & in)
{
  std::string out;
  for (auto itr = in.begin(); itr != in.end(); itr++) {
    if (*itr == ' ' || *itr == '-') {
      out += '_';
      continue;
    }
    out += tolower(*itr);
  }
  return out;
}

}  // namespace utils


/// Observer for monitoring task execution and completion by the scheduler
class SchedulerExecutionObserver : public task::ExecutionObserverBase
{
public:
  SchedulerExecutionObserver(
    std::shared_mutex & mtx,
    data::EventsHandler & eh,
    Scheduler & scheduler)
  : mtx_(mtx), eh_(eh), scheduler_(scheduler)
  {
  }

  void completion_callback(
    const std::string & id,
    bool success,
    const std::string & detail = "") final
  {
    nlohmann::json details_json;
    data::Event event;
    {  // Write Lock
      Scheduler::WriteLock lk(mtx_);
      RS_LOG_INFO("Calling scheduler completion callback");

      // Update completion details
      if (!eh_.has_event(id)) {
        return;
      }
      event = eh_.get_event(id);
      details_json = nlohmann::json::parse(event.event_details);
      details_json["success"] = success;
      details_json["completion_details"] = detail;

      // Also update event completion time
      uint64_t new_duration = utils::now() - event.start_time;
      if (!details_json.contains("completion_delay")) {
        details_json["completion_delay"] = 0;
      }
      if (new_duration > event.duration) {
        details_json["completion_delay"] =
          details_json["completion_delay"].get<int64_t>() +
          static_cast<double>(new_duration - event.duration) / 1e9;
      } else {
        details_json["completion_delay"] =
          details_json["completion_delay"].get<int64_t>() -
          static_cast<double>(event.duration - new_duration) / 1e9;
      }

      // Update event
      event.duration = new_duration;
      event.event_details = details_json.dump();
      eh_.update_event(event);
    }  // Write lock

    // Force the scheduler to tick
    scheduler_.tick_once();

    /***************** Dirty Fixes **************/
    if (!details_json.contains("fleet") || !details_json.contains("robot")) {
      return;
    }
    if (event.type == "rmf/robot_task") {
      return;
    }
    {
      Scheduler::WriteLock lk(mtx_);
      std::string fleet = utils::to_slug(details_json["fleet"].get<std::string>());
      std::string robot = utils::to_slug(details_json["robot"].get<std::string>());
      RS_LOG_INFO(
        "Task finished sending robot [%s/%s] to charger",
        fleet.c_str(), robot.c_str());
      // Check if there are events in the next 20min
      if (!scheduler_.enough_time_for_charging(
          fleet,
          robot,
          event.start_time + event.duration - 1,
          20 * 60 * 1e9))
      {
        return;
      }
      // Select the best one
      scheduler_.send_to_best_charger(fleet, robot);
    }
  }

  void update(
    const std::string & id,
    uint64_t remaining_time) final
  {
    data::Event event;
    uint64_t new_duration;
    {
      Scheduler::ReadLock lk(mtx_);
      // Update duration based on remaining_time
      if (!eh_.has_event(id)) {
        return;
      }
      event = eh_.get_event(id);
      new_duration = utils::now() - event.start_time + remaining_time;
      if (new_duration < event.duration) {
        RS_LOG_DEBUG("No need to update the duration");
        // No need to update if there is no dealy
        return;
      }
    }
    {  // Write Lock
      Scheduler::WriteLock lk(mtx_);

      // Update duration based on remaining_time
      auto details_json = nlohmann::json::parse(event.event_details);
      if (!details_json.contains("completion_delay")) {
        details_json["completion_delay"] = 0;
      }
      new_duration += static_cast<uint64_t>(10 * 60 * 1e9);  // Dirty fix add 10min
      double duration_increase = static_cast<double>(new_duration - event.duration) / 1e9;
      details_json["completion_delay"] =
        details_json["completion_delay"].get<int64_t>() + duration_increase;

      RS_LOG_WARN(
        "Task [%s] duration increased by %fs",
        event.id.c_str(),
        duration_increase);
      event.duration = new_duration;
      event.event_details = details_json.dump();
      eh_.update_event(event);
    }  // Write Lock

    // Force the scheduler to tick
    scheduler_.tick_once();
  }

private:
  std::shared_mutex & mtx_;
  data::EventsHandler & eh_;
  Scheduler & scheduler_;
};

Scheduler::Scheduler()
: Scheduler(SchedulerOptions())
{
}

Scheduler::Scheduler(const SchedulerOptions & options)
: options_(options)
{
  // Try loading old schedule from local cache files
  if (options_.enable_local_caching_) {
    cache_manager_ = std::make_shared<Cache>(
      options_.cache_keep_last_,
      options_.cache_dir_);

    // Initialize schedule from last cache
    cache_manager_->from_last_cache(schedule_);

    RS_LOG_INFO(
      "Event size: %lu",
      schedule_.events_handler().get_all_events().size());
  }

  // Initialize optimization configuration
  if (options_.enable_optimization_) {
    std::regex day_expr_regex("^[1-3][0-9]?d$");

    // Check if it is in the format of "Xd"
    if (std::regex_match(options_.optimization_window_, day_expr_regex)) {
      // Get the starting number
      std::string day_expr = options_.optimization_window_;
      day_expr = day_expr.substr(0, day_expr.size() - 1);
      RS_LOG_INFO("Optimization enabled in the period of %s days", day_expr.c_str());

      // Construct cron
      std::string day_cron_expr = "0 0 0 */" + day_expr + " * *";
      window_utils_ = std::make_shared<WindowUtils>(
        day_cron_expr, options_.optimization_window_timezone_);
    } else {
      RS_LOG_ERROR("Unable to enable optimization");
      options_.enable_optimization_ = false;
    }
  }

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

  // Initialize notification manager
  notification_manager_ = rmf_notification::NotificationManager::get();

  schema_validator_ = std::make_unique<SchemaValidator>(schemas);

  last_write_time_ = utils::now();

  series_expand_time_ = utils::now();

  task_executor_.make_observer<SchedulerExecutionObserver>(
    mtx_, schedule_.events_handler(), *this);

  // Runtime parameters
  spinning_ = false;
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

  // Reject tasks in the past
  for (auto & event : events_to_add) {
    if (event.second.start_time < read_time -
      static_cast<uint64_t>(
        options_.allow_past_events_duration_ * 1e9))
    {
      throw exception::ScheduleWriteToPastException(
              "Event [%s] is in the past, cannot add.",
              event.second.id.c_str());
    }
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
      options_.estimate_timeout_);


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
    task_estimator_.estimate(events_to_update, options_.estimate_timeout_);

    // Update schedule cache
    // Update events first
    for (auto & event_itr : events_to_update) {
      // Event runtime update
      event_runtime_update(event_itr.first);

      // Update event
      schedule_.update_event(event_itr.second);
    }

    // Update the dependencies
    for (auto & dag_itr : dags_to_update) {
      // DAG runtime updates
      dag_runtime_update(dag_itr.first);

      // Update DAG
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

  // Rejcet if the task to be deleted is ongoing
  for (auto event_id : event_ids) {
    if (task_executor_.is_ongoing(event_id)) {
      throw exception::ScheduleWriteToPastException(
              "Event [%s] is ongoing, cannot delete.",
              event_id.c_str());
    }
  }

  {  // Write lock
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

      // Initiate runtime update
      if (is_dag) {
        dag_runtime_update(event.dag_id);
      } else {
        event_runtime_update(event.id);
      }

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
      // Run DAG runtime update
      dag_runtime_update(dag_id);
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
  }  // Write lock

  // tick once since some events are deleted
  if (spinning_) {
    _tick();
  }
}

const data::Schedule & Scheduler::schedule_handler_const() const
{
  return schedule_;
}

data::Schedule & Scheduler::schedule_handler()
{
  return schedule_;
}

void Scheduler::event_runtime_update(
  const std::string & event_id)
{
  // Check if the event id is pushed
  auto event_action_id_itr = pushed_action_ids_.find(event_id);
  if (event_action_id_itr != pushed_action_ids_.end()) {
    RS_LOG_INFO(
      "Removing event [%s] from queue due to changes",
      event_id.c_str());
    // Remove the event from the system time executor
    ste_.delete_action(event_id);
    pushed_action_ids_.erase(event_action_id_itr);
  }
}

void Scheduler::dag_runtime_update(
  const std::string & dag_id)
{
  // Check if DAG Executor is created and ongoing
  auto dag_executor_itr = dag_executors_.find(dag_id);
  if (dag_executor_itr != dag_executors_.end()) {
    dag_executor_itr->second->cancel();
    RS_LOG_INFO(
      "DAG event [%s] changed, finishing remaining task first",
      dag_executor_itr->first.c_str());
  }

  // Check if the dag id is pushed
  auto dag_action_id_itr = pushed_action_ids_.find(dag_id);
  if (dag_action_id_itr != pushed_action_ids_.end()) {
    // Add tasks executed to the push_ids
    for (auto & event_id : schedule_.get_dag(dag_id).all_nodes()) {
      if (task_executor_.is_ongoing(event_id) ||
        task_executor_.is_completed(event_id))
      {
        RS_LOG_INFO(
          "DAG event [%s] already pushed, keep an record due to exeuction change.",
          event_id.c_str());
      }
    }

    // remove the dag from the ste as well
    ste_.delete_action(dag_id);
    // Erase the DAG id as well
    pushed_action_ids_.erase(dag_action_id_itr);
  }
}

void Scheduler::optimize(uint64_t start_time, uint64_t end_time)
{
  RS_LOG_INFO("Evaluate need to re-estimate.");
  // Get all events to optimize
  auto events_to_optimize =
    schedule_.events_handler_const().lookup_events(
    start_time,
    end_time);

  // Check if some of the events are ongoing, no need to re-estimate if so
  std::vector<data::Event> ongoing_events;
  std::vector<std::string> ongoing_event_ids;
  auto event_itr = events_to_optimize.begin();
  while (event_itr != events_to_optimize.end()) {
    if (task_executor_.is_ongoing(event_itr->id) ||
      task_executor_.is_completed(event_itr->id))
    {
      // Add them to ongoing event
      ongoing_events.push_back(*event_itr);
      ongoing_event_ids.push_back(event_itr->id);
      RS_LOG_INFO("Ongoing event found: %s", event_itr->id.c_str());
      // remove the events from events_to_optimize
      event_itr = events_to_optimize.erase(event_itr);
    } else {
      event_itr++;
    }
  }

  // Retrieve the DAG involved
  // And detach the events
  std::unordered_set<std::string> dag_ids;
  for (auto & event : events_to_optimize) {
    if (event.dag_id.empty()) {
      if (!event.series_id.empty()) {
        // detach the event
        RS_LOG_INFO("Detaching event");
        schedule_.detach_event_series_occurrence(event.series_id, event.id);
      }
      continue;
    }

    if (dag_ids.find(event.dag_id) == dag_ids.end()) {
      RS_LOG_INFO("adding dag %s", event.dag_id.c_str());
      dag_ids.emplace(event.dag_id);
      if (!event.series_id.empty()) {
        RS_LOG_INFO("Detaching dag");
        schedule_.detach_dag_series_occurrence(event.series_id, event.dag_id);
      }
    }
  }

  // Refresh the DAG
  for (auto & dag_id : dag_ids) {
    RS_LOG_INFO("adding dag timing %s", dag_id.c_str());
    schedule_.generate_dag_event_start_time(dag_id);
  }

  // Update the events quickly
  for (size_t i = 0; i < events_to_optimize.size(); i++) {
    // Get the updated event
    events_to_optimize[i] = schedule_.get_event(events_to_optimize[i].id);

    // Remove "conflict" in the event details
    try {
      auto details = nlohmann::json::parse(events_to_optimize[i].event_details);
      if (details.contains("conflict")) {
        details.erase("conflict");
      }
      events_to_optimize[i].event_details = details.dump();
      schedule_.update_event(events_to_optimize[i]);
    } catch (const std::exception & e) {
      continue;
    }
  }

  // Store all the events start / end state into the estimator
  task_estimator_.clear_estimate_states();
  task_estimator_.add_estimate_states(events_to_optimize);

  if (!task_estimator_.validate_all_estimate_states()) {
    RS_LOG_INFO("Existing states no longer valid, re-estimating...");
    // Re-estimate the events when estimator states not valid
    std::unordered_map<std::string, data::Event> events_to_estimate;
    for (auto & event : events_to_optimize) {
      events_to_estimate[event.id] = event;
    }

    task_estimator_.clear_estimate_states();
    task_estimator_.estimate(events_to_estimate, options_.estimate_timeout_);

    // Update the event duration
    for (auto & itr : events_to_estimate) {
      schedule_.update_event(itr.second);
    }

    // Refresh the DAG again
    for (auto & dag_id : dag_ids) {
      schedule_.generate_dag_event_start_time(dag_id);
    }

    // Update the event start time and duration
    for (size_t i = 0; i < events_to_optimize.size(); i++) {
      // Get the updated event
      events_to_optimize[i] = schedule_.get_event(events_to_optimize[i].id);
    }
  } else {
    RS_LOG_INFO("No need to re-estimate.");
  }

  // Rollback window one step to find event
  auto rollback_window = window_utils_->get_window(end_time + 1);
  rollback_window.start_time -= rollback_window.end_time - end_time;
  rollback_window.end_time = start_time;
  auto events_to_identify =
    schedule_.events_handler_const().lookup_events(
    rollback_window.start_time,
    rollback_window.end_time);

  // TODO(Briancbn): handle old untracked events
  // Check if there are past events that needs to addded in for conflict checking
  for (auto & event : events_to_identify) {
    if (event.start_time + event.duration >= start_time) {
      event.duration = event.start_time + event.duration - start_time;
      event.start_time = start_time;
      RS_LOG_INFO("Additional ongoing event found: %s", event.id.c_str());
      ongoing_events.push_back(event);
      ongoing_event_ids.push_back(event.id);
    }
  }

  // Push back ongoing events too
  std::vector<data::Event> events_to_optimize_w_ongoing = events_to_optimize;
  for (auto & event : ongoing_events) {
    events_to_optimize_w_ongoing.push_back(event);
  }

  // Get events to check conflict
  RS_LOG_INFO("Checking for conflict:\n");
  auto initial_conflicts = utils::identify_conflicts(
    events_to_optimize_w_ongoing, {}, {"robot", "zone"});

  if (initial_conflicts.empty()) {
    RS_LOG_INFO("No conflict");
    return;
  }

  // Run optimization pipeline
  RS_LOG_INFO("Conflict detected start deconflictioni size: %lu", initial_conflicts.size());
  // Initialize the solver
  auto cp_solver = conflict::CpSolver::make();

  // Get a bigger window for resolution
  auto new_window = window_utils_->get_window(end_time);

  // Use additional 0.5 of the next optimization window
  new_window.end_time -= (new_window.end_time - new_window.start_time) / 2;
  new_window.start_time = start_time;
  cp_solver->init(
    events_to_optimize_w_ongoing, new_window, 5.0);

  // Categorise the events by type
  auto event_by_type = utils::categorise_by_type(events_to_optimize);

  // Mark flight schedule as fixed
  auto flight_event_itr = event_by_type.find("flight-schedule");
  if (flight_event_itr != event_by_type.end()) {
    cp_solver->mark_fixed(flight_event_itr->second);
  }

  // Mark Vendor maintenance as fixed
  auto vm_itr = event_by_type.find("Vendor Maintenance");
  if (vm_itr != event_by_type.end()) {
    cp_solver->mark_fixed(vm_itr->second);
  }

  // Mark Daily maintenance as fixed
  auto dm_itr = event_by_type.find("Daily Maintenance");
  if (dm_itr != event_by_type.end()) {
    cp_solver->mark_fixed(dm_itr->second);
  }

  // Mark ongoing event as fixed
  cp_solver->mark_fixed(ongoing_event_ids);

  // Add robot task to the objective function
  auto robot_event_itr = event_by_type.find("Cleaning Task");
  if (robot_event_itr != event_by_type.end()) {
    cp_solver->add_objective(robot_event_itr->second);
  }

  // Solve for a feasible solution
  auto event_by_filter = utils::categorise_by_filter(
    events_to_optimize_w_ongoing, {"robot", "zone"});

  for (auto & itr : event_by_filter) {
    for (auto & itr2 : itr) {
      cp_solver->add_no_overlap(itr2.second);
    }
  }

  // Solve for a way to avoid conflict
  auto result = cp_solver->solve(true);

  if (result.empty()) {
    RS_LOG_INFO("Update details");
    // Mark the events in conflict
    for (auto & conflict : initial_conflicts) {
      RS_LOG_INFO("Update details");
      auto first_event = schedule_.get_event(conflict.first);
      auto second_event = schedule_.get_event(conflict.second);
      // Add conflict in the event details
      try {
        auto first_details = nlohmann::json::parse(first_event.event_details);
        auto second_details = nlohmann::json::parse(second_event.event_details);
        first_details["conflict"] = true;
        second_details["conflict"] = true;
        first_event.event_details = first_details.dump();
        second_event.event_details = second_details.dump();
        RS_LOG_INFO(
          "first event: start_time - %lu, end_time - %lu.", first_event.start_time,
          first_event.start_time + first_event.duration);
        RS_LOG_INFO(
          "second event event: start_time - %lu, end_time - %lu.",
          second_event.start_time, second_event.start_time + second_event.duration);
      } catch (const std::exception & e) {
        RS_LOG_ERROR("Cannot parse details");
        continue;
      }
      schedule_.update_event(first_event);
      schedule_.update_event(second_event);
    }
  }

  RS_LOG_INFO("Solver completed successfully");
  // Implement the changes
  std::unordered_map<std::string, std::unordered_map<std::string, data::Event>>
  dag_events_to_update;
  std::unordered_map<std::string, std::unordered_map<std::string, data::Event>>
  dag_events_to_remove;
  for (auto & change : result) {
    RS_LOG_INFO(
      "Event id: %s\n"
      "orig: %lus, final: %lus.",
      change.id.c_str(),
      change.original_start_time,
      change.final_start_time);
    auto event = schedule_.get_event(change.id);

    // Check if the event exceeds the deadline
    if (change.final_start_time + event.duration > end_time) {
      RS_LOG_ERROR(
        "Event [%s] cannot be executed before %s",
        change.id.c_str(),
        utils::to_localtime(end_time));

      std::string message = event.description + " cannot be executed at [" +
        utils::to_localtime(end_time) +
        "]\nClick the link below for more information:";
      notification_manager_->publish(message, "maintenance_log_update");

      // Check if events is part of the DAG
      if (event.dag_id.empty()) {
        // Remove the event
        if (event.series_id.empty()) {
          schedule_.delete_event(event.id);
        } else {
          schedule_.delete_event_series_occurrence(event.series_id, event.id);
        }

        // Update event runtime info
        event_runtime_update(event.id);
      } else {
        dag_events_to_remove[event.dag_id].emplace(event.id, event);
      }
    }

    // Update normally
    event.start_time = change.final_start_time;
    if (event.dag_id.empty()) {
      schedule_.update_event(event);

      // Update event runtime info
      event_runtime_update(event.id);
    } else {
      dag_events_to_update[event.dag_id].emplace(event.id, event);
    }
  }

  // Update the DAG, assuming it's a sequence
  // TODO(anyone): fix this for real DAG
  for (auto & itr : dag_events_to_update) {
    // First get a duplicate of the dag
    std::string dag_id = itr.first;

    // Update DAG runtime info
    dag_runtime_update(dag_id);
    const data::DAG & old_dag = schedule_.dags()[dag_id];
    data::DAG updated_old_dag;
    std::vector<data::DAG> new_dags;
    auto entry_nodes = schedule_.dags_const().at(itr.first).entry_nodes();

    if (entry_nodes.size() != 1) {
      RS_LOG_WARN("Multiple entry nodes not supported yet, for update");
      // Break everything up
      continue;
    }

    auto first_node = *entry_nodes.begin();
    const auto & events_to_update = itr.second;
    std::unordered_map<std::string, data::Event> events_to_remove;
    auto events_to_remove_itr = dag_events_to_remove.find(dag_id);
    if (events_to_remove_itr != dag_events_to_remove.end()) {
      events_to_remove = events_to_remove_itr->second;
    }

    _generate_new_dags_recursive(
      {first_node},
      updated_old_dag,
      old_dag,
      new_dags,
      "",
      0,
      events_to_update,
      events_to_remove);

    // Update the schedules
    for (auto & itr : events_to_update) {
      schedule_.update_event(itr.second);
    }
    schedule_.update_dag(dag_id, std::move(updated_old_dag));

    for (size_t i = 0; i < new_dags.size(); i++) {
      // Add the additional DAGs
      schedule_.add_dag(dag_id + '-' + std::to_string(i), std::move(new_dags[i]));
    }

    // Remove the ids
    for (auto & itr : events_to_remove) {
      schedule_.delete_event(itr.second.id);
    }
  }

  schedule_.purge();
}

void Scheduler::_generate_new_dags_recursive(
  const data::DAG::DependencyInfo & info,
  data::DAG & dag_to_extend,
  const data::DAG & old_dag,
  std::vector<data::DAG> & new_dags,
  const std::string & previous_node,
  int64_t previous_time_change,
  const std::unordered_map<std::string, data::Event> & events_to_update,
  const std::unordered_map<std::string, data::Event> & events_to_remove)
{
  for (auto & successor_id : info) {
    RS_LOG_DEBUG("Refactor dag event [%s]", successor_id.c_str());
    auto new_successor_info = old_dag.get_successor_info(successor_id);
    auto all_node = old_dag.all_nodes();

    // Check if event is to be removed
    auto remove_itr = events_to_remove.find(successor_id);
    if (remove_itr != events_to_remove.end()) {
      RS_LOG_DEBUG("event[%s], will be removed", successor_id.c_str());
      if (new_successor_info.empty()) {
        RS_LOG_DEBUG("DAG refactor recursive end");
        continue;
      }

      // Check if previous node is to be removed too
      if (!previous_node.empty()) {
        RS_LOG_DEBUG("Starting a new DAG");
        data::DAG & new_dag_to_extend = new_dags.emplace_back();
        _generate_new_dags_recursive(
          new_successor_info,
          new_dag_to_extend,
          old_dag,
          new_dags,
          "", 0, events_to_update, events_to_remove);
        continue;
      } else {
        RS_LOG_DEBUG("Continuing the new DAG");
        _generate_new_dags_recursive(
          new_successor_info,
          dag_to_extend,
          old_dag,
          new_dags,
          "", 0, events_to_update, events_to_remove);
        continue;
      }
    }

    // Check if dependency is broken
    bool dep_broken = false;
    int64_t dep_time_change = 0;
    auto update_itr = events_to_update.find(successor_id);
    if (update_itr == events_to_update.end()) {
      if (previous_time_change == 0) {
        RS_LOG_DEBUG("DAG continuing since no change in start time");
        dep_broken = false;
      } else {
        RS_LOG_DEBUG("DAG broken due to parent updated, child no change");
        dep_broken = true;
      }
    } else {
      dep_time_change =
        update_itr->second.start_time - schedule_.get_event(successor_id).start_time;

      dep_broken = !(dep_time_change == previous_time_change);
      RS_LOG_DEBUG("DAG broken? %s", dep_broken ? "true" : "false");
    }

    if (previous_node.empty()) {
      RS_LOG_DEBUG("Add first node of a new DAG graph");
      dag_to_extend.add_node(successor_id);
      _generate_new_dags_recursive(
        new_successor_info,
        dag_to_extend,
        old_dag,
        new_dags,
        successor_id, dep_time_change, events_to_update, events_to_remove);
      continue;
    }

    if (dep_broken) {
      // Create a new dag to pass along
      RS_LOG_DEBUG("Starting a new DAG graph");
      data::DAG & new_dag_to_extend = new_dags.emplace_back();
      new_dag_to_extend.add_node(successor_id);
      _generate_new_dags_recursive(
        new_successor_info,
        new_dag_to_extend,
        old_dag,
        new_dags,
        successor_id, dep_time_change, events_to_update, events_to_remove);
    } else {
      // Add a tree item to the previous node
      dag_to_extend.add_node(successor_id);
      dag_to_extend.add_dependency(successor_id, {previous_node});
      _generate_new_dags_recursive(
        new_successor_info,
        dag_to_extend,
        old_dag,
        new_dags,
        successor_id, dep_time_change, events_to_update, events_to_remove);
      RS_LOG_DEBUG("Continuing previous DAG graph");
    }
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

void Scheduler::tick_once()
{
  if (!spinning_) {
    throw exception::TaskExecutionException("tick_once() can only be called after spin()");
  }
  _tick();
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
  RS_LOG_INFO(
    "Ticking Current Time: %s",
    utils::to_localtime(current_time));

  // Only tick when past the next tick time
  if (next_tick_time_ <= current_time) {
    next_tick_time_ = next_tick_time_ + static_cast<uint64_t>(tick_period_) * 1e9;

    RS_LOG_INFO(
      "Add next timer tick at time %s",
      utils::to_localtime(next_tick_time_));

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
    static_cast<uint64_t>(options_.allow_past_events_duration_ * 1e9);
  uint64_t tick_period_end_time = next_tick_time_;

  // Run the optimization pipeline
  if (options_.enable_optimization_) {
    auto optimization_window = window_utils_->get_window(utils::now());
    uint64_t allow_past =
      static_cast<uint64_t>(options_.allow_past_events_duration_ * 1e9);

    // Run optimize
    optimize(
      optimization_window.start_time - allow_past,
      optimization_window.end_time);
  }

  // Push these events to the queue
  _push_events(tick_period_start_time, tick_period_end_time);

  // Update local cache
  if (options_.enable_local_caching_) {
    RS_LOG_INFO("Updating local cache");
    cache_manager_->write_local_cache(schedule_);
  }

  // TODO(Briancbn): iterate until valid

  // Prune unused DAG executor
  for (auto dag_executor_itr = dag_executors_.begin();
    dag_executor_itr != dag_executors_.end(); )
  {
    if (!dag_executor_itr->second->ongoing()) {
      dag_executor_itr = dag_executors_.erase(dag_executor_itr);
    } else {
      dag_executor_itr++;
    }
  }
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
  std::unordered_set<std::string> dags_detached;
  for (auto & event : events) {
    // Detach event if they are part of a series
    if (!event.series_id.empty()) {
      if (event.dag_id.empty()) {
        schedule_.detach_event_series_occurrence(event.series_id, event.id);
      } else {
        // Check if the dag is already detached
        if (dags_detached.find(event.dag_id) == dags_detached.end()) {
          schedule_.detach_dag_series_occurrence(event.series_id, event.dag_id);
          dags_detached.emplace(event.dag_id);
        }
      }
    }
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
      action = _generate_dag_action(start_time, action_id, dag.description());
      ste_.add_action(dag_event, action);
    } else {
      action = _generate_event_action(event);
      ste_.add_action(event, action);
    }
    pushed_action_ids_.emplace(action_id);
  }
}

runtime::SystemTimeExecutor::Action Scheduler::_generate_dag_action(
  uint64_t start_time,
  const std::string & dag_id,
  const data::DAG::Description & dag)
{
  auto work_generator =
    [ = ](const std::string & id) -> runtime::DAGExecutor::Work {
      if (task_executor_.is_ongoing(id) ||
        task_executor_.is_completed(id))
      {
        RS_LOG_INFO(
          "Skipping [%s] in DAG due to ongoing / completed",
          id.c_str());
        return runtime::DAGExecutor::EmptyWork();
      }

      data::Event event = schedule_.get_event(id);
      if (event.start_time < start_time) {
        RS_LOG_INFO(
          "Skipping [%s] in DAG due to in the past",
          id.c_str());
        return runtime::DAGExecutor::EmptyWork();
      }
      return _generate_event_action(event, true);
    };

  return [ = ]() -> void {
           // Write lock
           WriteLock lk(mtx_);
           // Check if DAG executor under the same id exist
           auto dag_executor_itr = dag_executors_.find(dag_id);
           if (dag_executor_itr != dag_executors_.end()) {
             RS_LOG_INFO(
               "DAG [%s] already running, "
               "updating existing execution instead",
               dag_id.c_str());
             dag_executor_itr->second->cancel_and_next(dag, work_generator);
           } else {
             RS_LOG_INFO(
               "DAG [%s] created in execution.",
               dag_id.c_str());
             // Create a new one if doesn't exist
             auto dag_executor = std::make_unique<runtime::DAGExecutor>();
             auto future = dag_executor->run(dag, work_generator);
             dag_executors_.emplace(dag_id, std::move(dag_executor));
           }
         };
}

runtime::SystemTimeExecutor::Action Scheduler::_generate_event_action(
  const data::Event & event,
  bool blocking)
{
  return [ = ]() {
           std::shared_future<task::Executor::Status> future;
           data::Event orig_event = event;


           /***************** Dirty Fixes **************/
           auto temp_details_json =
             nlohmann::json::parse(orig_event.event_details);
           if (temp_details_json.contains("robot") &&
             temp_details_json.contains("fleet"))
           {
             std::string fleet = utils::to_slug(temp_details_json["fleet"].get<std::string>());
             std::string robot = utils::to_slug(temp_details_json["robot"].get<std::string>());
             update_left_charger(fleet, robot);
             RS_LOG_INFO(
               "Robot [%s/%s] has left its charger",
               fleet.c_str(), robot.c_str());
           }
           /***************** Dirty Fixes **************/
           try {
             future = task_executor_.run_async(event);
           } catch (const std::exception & e) {
             // Report premature failure
             Scheduler::WriteLock lk(mtx_);

             // Update completion details
             data::Event new_event = schedule_.get_event(orig_event.id);
             auto details_json = nlohmann::json::parse(orig_event.event_details);
             details_json["success"] = false;
             details_json["completion_details"] = e.what();
             RS_LOG_ERROR(
               "Event [%s] execution start-up error, "
               "please check the execution plugin for event type [%s], "
               "skipping execution...\n"
               "Error Message: %s",
               orig_event.id.c_str(),
               orig_event.type.c_str(),
               e.what());
             new_event.event_details = details_json.dump();

             schedule_.update_event(new_event);
           }
           if (blocking) {
             future.wait();
           }
         };
}

/***************** Dirty Fixes **************/
bool Scheduler::enough_time_for_charging(
  const std::string & fleet_name,
  const std::string & robot_name,
  uint64_t start_time,
  uint64_t allowed_duration)
{
  auto events = schedule_.events_handler_const().lookup_events(
    start_time, start_time + allowed_duration);
  RS_LOG_INFO("size: %lu", events.size());
  for (auto & event : events) {
    nlohmann::json details = nlohmann::json::parse(event.event_details);
    RS_LOG_INFO("Checking fields");
    if (details.contains("fleet") && details.contains("robot")) {
      std::string fleet = utils::to_slug(details["fleet"].get<std::string>());
      std::string robot = utils::to_slug(details["robot"].get<std::string>());
      RS_LOG_INFO(
        "fleet: %s, fleet_name: %s, robot: %s, robot_name: %s",
        fleet.c_str(), fleet_name.c_str(), robot.c_str(), robot_name.c_str());
      if (fleet == fleet_name && robot == robot_name) {
        RS_LOG_WARN(
          "Upcoming task [%s] for [%s/%s], no charging",
          event.id.c_str(),
          fleet_name.c_str(),
          robot_name.c_str());
        return false;
      }
    }
  }
  return true;
}

void Scheduler::send_to_best_charger(
  const std::string & fleet_name,
  const std::string & robot_name)
{
  // Get the charger(s)
  std::string charger_name;
  auto itr = options_.dynamic_charger_map_.find(fleet_name);
  if (itr == options_.dynamic_charger_map_.end()) {
    RS_LOG_WARN(
      "No dynamic charger set for for [%s/%s], checking fixed charger",
      fleet_name.c_str(),
      robot_name.c_str());

    auto fixed_itr = options_.fixed_charger_map_.find(fleet_name + '/' + robot_name);
    if (fixed_itr == options_.fixed_charger_map_.end()) {
      RS_LOG_WARN(
        "No fixed charger set for for [%s/%s]. No charging",
        fleet_name.c_str(),
        robot_name.c_str());
      return;
    }
    charger_name = fixed_itr->second;
  } else {
    // Dynamic allocation
    uint64_t duration = UINT64_MAX;
    for (auto & dynamic_itr : itr->second) {
      if (!dynamic_itr.second.empty()) {
        continue;
      }
      // Create a go to place to estimate
      nlohmann::json task_detail;
      task_detail["type"] = "robot_task_request";
      task_detail["fleet"] = fleet_name;
      task_detail["robot"] = robot_name;
      nlohmann::json go_to_place_activity;
      go_to_place_activity["category"] = "go_to_place";
      go_to_place_activity["description"]["waypoint"] = dynamic_itr.first;

      nlohmann::json phase;
      phase["activity"] = go_to_place_activity;

      nlohmann::json request;
      request["category"] = "compose";
      request["description"]["category"] = "go_to_place";
      request["description"]["phases"] = nlohmann::json::array();
      request["description"]["phases"].push_back(phase);
      task_detail["request"] = request;

      data::Event charge_event;
      charge_event.id = "charge-" + utils::gen_uuid();
      charge_event.start_time = utils::now();
      charge_event.duration = 0;  // random
      charge_event.type = "rmf/robot_task";
      charge_event.task_details = task_detail.dump();

      std::unordered_map<std::string, data::Event> estimate_request;
      estimate_request[charge_event.id] = charge_event;
      // Estimate
      task_estimator_.estimate(
        estimate_request, options_.estimate_timeout_);

      // Compare with duration
      uint64_t new_duration = estimate_request.at(charge_event.id).duration;
      if (new_duration == 0) {
        RS_LOG_WARN(
          "Estimation failed [%s/%s]. Use the first available waypoint",
          fleet_name.c_str(),
          robot_name.c_str());
        charger_name = dynamic_itr.first;
        break;
      } else if (new_duration < duration) {
        charger_name = dynamic_itr.first;
        duration = new_duration;
      }
      RS_LOG_INFO(
        "[%s/%s] going to charger [%s] will take %fs",
        fleet_name.c_str(),
        robot_name.c_str(),
        dynamic_itr.first.c_str(),
        static_cast<double>(new_duration) / 1e9);
    }
    if (charger_name.empty()) {
      RS_LOG_WARN(
        "No dynamic charger set for for [%s/%s]. No charging",
        fleet_name.c_str(),
        robot_name.c_str());
      return;
    }

    itr->second[charger_name] = robot_name;
  }

  // Send the robot to charger
  RS_LOG_INFO(
    "Sending [%s/%s] to charger [%s]",
    fleet_name.c_str(),
    robot_name.c_str(),
    charger_name.c_str());
  // Create a go to place task for charging
  nlohmann::json task_detail;
  task_detail["type"] = "robot_task_request";
  task_detail["fleet"] = fleet_name;
  task_detail["robot"] = robot_name;
  nlohmann::json go_to_place_activity;
  go_to_place_activity["category"] = "go_to_place";
  go_to_place_activity["description"]["waypoint"] = charger_name;

  nlohmann::json phase;
  phase["activity"] = go_to_place_activity;

  nlohmann::json request;
  request["category"] = "compose";
  request["description"]["category"] = "go_to_place";
  request["description"]["phases"] = nlohmann::json::array();
  request["description"]["phases"].push_back(phase);
  task_detail["request"] = request;

  data::Event charge_event;
  charge_event.id = "charge-" + utils::gen_uuid();
  charge_event.start_time = utils::now();
  charge_event.duration = 0;  // random
  charge_event.type = "rmf/robot_task";
  charge_event.task_details = task_detail.dump();

  task_executor_.run_async(charge_event);
}

void Scheduler::update_left_charger(
  const std::string & fleet_name,
  const std::string & robot_name)
{
  auto itr = options_.dynamic_charger_map_.find(fleet_name);
  if (itr == options_.dynamic_charger_map_.end()) {
    return;
  }
  for (auto & charger_itr : itr->second) {
    if (charger_itr.second == robot_name) {
      charger_itr.second.clear();
    }
  }
}

/***************** Dirty Fixes **************/
}  // namespace rmf_scheduler
