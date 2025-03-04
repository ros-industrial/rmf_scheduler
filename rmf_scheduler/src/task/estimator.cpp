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

#include "rmf_scheduler/task/estimator.hpp"
#include "rmf_scheduler/task/task_plugin_impl.hpp"
#include "rmf_scheduler/utils/uuid.hpp"
#include "rmf_scheduler/parser.hpp"

namespace rmf_scheduler
{

namespace task
{

Estimator::Estimator()
{
}

Estimator::~Estimator()
{
}

void Estimator::load_plugin(
  const std::shared_ptr<void> & node,
  const std::string & name,
  const std::string & interface,
  const std::vector<std::string> & supported_tasks)
{
  load_task_plugin_impl<EstimateInterface>(
    this, node, name, interface, "rmf_scheduler::task::EstimateInterface", supported_tasks);
}

void Estimator::unload_plugin(
  const std::string & name)
{
  TaskPluginManager::unload_plugin(name);
}

void Estimator::estimate(
  std::unordered_map<std::string, data::Event> & events_to_add,
  double timeout)
{
  // TODO(anyone): robot allocation?
  // Align the event based on robot name and their start time
  std::unordered_map<std::string, std::multimap<uint64_t, data::Event>> event_lookup;
  for (auto & event_itr : events_to_add) {
    // Check if there is an estimation interface
    if (!is_supported(event_itr.second.type)) {
      RS_LOG_DEBUG("event type not supported: %s", event_itr.second.type.c_str());
      continue;
    }

    // Filter out robot information
    std::string robot_name;
    bool result = parser::filter_event_details<std::string>(
      event_itr.second.task_details,
      "robot",
      robot_name
    );
    if (!result) {
      RS_LOG_WARN(
        "Invalid task detail for %s[%s], skipping estimation",
        event_itr.second.id.c_str(),
        event_itr.second.type.c_str());
      continue;
      // TODO(anyone): robot maybe?
    }

    event_lookup[robot_name].emplace(
      event_itr.second.start_time, event_itr.second);
  }

  // Go through the order based on start time and existing estimation state
  for (auto & robot_event_itr : event_lookup) {
    // uint64_t last_estimated_event_end_time = 0;
    std::shared_ptr<EstimateState> last_estimated_event_end_state;

    // Check if there are existing estimated events under the same robot

    for (auto & event_itr : robot_event_itr.second) {
      auto & event = event_itr.second;

      // Retrieve the estimation interface
      auto estimate_interface = get_supported_plugin<EstimateInterface>(
        event_itr.second.type).second;
      // Retrieve the task details
      nlohmann::json task_details = nlohmann::json::parse(event.task_details);

      // start estimation
      std::shared_future<EstimateResponse> future;
      EstimateRequest request;
      request.start_time = event.start_time;
      request.details = task_details;
      request.state = last_estimated_event_end_state.get();
      try {
        future = estimate_interface->async_estimate(
          utils::gen_uuid(),
          request);
      } catch (const std::exception & e) {
        throw exception::InvalidEstimateInterfaceException(
                "Cannot estimate task [%s]:\n %s\n error: %s",
                event.id.c_str(),
                event.event_details.c_str(),
                e.what());
      }

      // Check if the future is valid
      if (!future.valid()) {
        RS_LOG_ERROR("Future invalid");
        continue;
      }
      if (timeout > 0) {
        if (future.wait_for(std::chrono::duration<double>(timeout)) !=
          std::future_status::ready)
        {
          RS_LOG_ERROR("Estimation timeout");
          continue;
        }
      }

      auto result = future.get();
      RS_LOG_INFO("Received: %s", result.state.json().dump(2).c_str());

      // Modify the real event
      uint64_t duration = result.duration;
      data::Event & orig_event = events_to_add[event.id];
      orig_event.duration = duration;
      task_details["end_state"] = result.state.json();
      if (last_estimated_event_end_state != nullptr) {
        task_details["start_state"] = last_estimated_event_end_state->json();
      }
      orig_event.task_details = task_details.dump();
      // last_estimated_event_end_time = event.start_time + duration;

      // Update last estimated end state
      last_estimated_event_end_state = std::make_shared<EstimateState>(result.state);
    }
  }
}


bool Estimator::validate_all_estimate_states() const
{
  for (auto & itr : robot_estimate_states_) {
    if (!validate_estimate_states(itr.first)) {
      return false;
    }
  }
  return true;
}


bool Estimator::validate_estimate_states(
  const std::string & robot_name,
  uint64_t start_time,
  uint64_t end_time) const
{
  if (start_time >= end_time) {
    // valid
    return true;
  }

  const EstimateState * last_event_end_state = nullptr;

  auto itr_to_validate = robot_estimate_states_.find(robot_name);
  if (itr_to_validate == robot_estimate_states_.end()) {
    return false;
  }

  auto itr = itr_to_validate->second.lower_bound(start_time);
  auto upper_itr = itr_to_validate->second.upper_bound(end_time);
  for (; itr != upper_itr; itr++) {
    if (!itr->second.start || !itr->second.end) {
      return false;
    }

    if (last_event_end_state != nullptr &&
      *last_event_end_state != *itr->second.start)
    {
      return false;
    }

    last_event_end_state = itr->second.end.get();
  }
  return true;
}

void Estimator::add_estimate_states(
  const std::string & robot_name,
  uint64_t time,
  const EstimateStates & estimated_state)
{
  robot_estimate_states_[robot_name].emplace(time, estimated_state);
}

void Estimator::add_estimate_states(
  const std::vector<data::Event> & events)
{
  for (auto & event : events) {
    // Filter out robot information
    std::string robot_name;
    bool result = parser::filter_event_details<std::string>(
      event.task_details,
      "robot",
      robot_name
    );
    if (!result) {
      continue;
    }

    nlohmann::json task_details;
    try {
      task_details = nlohmann::json::parse(event.task_details);
    } catch (const std::exception & e) {
      RS_LOG_ERROR(
        "Invalid [%s] task details JSON, cannot load estimate info.%s",
        event.id.c_str(),
        e.what());
      continue;
    }

    EstimateStates states;

    // Load start state
    if (task_details.contains("start_state")) {
      try {
        states.start = EstimateState::from_json(task_details["start_state"]);
      } catch (const std::exception & e) {
        RS_LOG_ERROR(
          "Unable to load [%s] estimate start_state info.%s",
          event.id.c_str(),
          e.what());
        continue;
      }
    }

    if (task_details.contains("end_state")) {
      try {
        states.end = EstimateState::from_json(task_details["end_state"]);
      } catch (const std::exception & e) {
        RS_LOG_ERROR(
          "Unable to load [%s] estimate end_state info.%s",
          event.id.c_str(),
          e.what());
        continue;
      }
    }
    add_estimate_states(robot_name, event.start_time, states);
  }
}

void Estimator::delete_estimate_states(
  const std::string & robot_name,
  uint64_t start_time,
  uint64_t end_time)
{
  auto itr_to_delete = robot_estimate_states_.find(robot_name);
  if (itr_to_delete == robot_estimate_states_.end()) {
    return;
  }
  auto lower_itr = itr_to_delete->second.lower_bound(start_time);
  auto upper_itr = itr_to_delete->second.lower_bound(end_time);
  itr_to_delete->second.erase(lower_itr, upper_itr);
}

void Estimator::clear_estimate_states()
{
  robot_estimate_states_.clear();
}

}  // namespace task

}  // namespace rmf_scheduler
