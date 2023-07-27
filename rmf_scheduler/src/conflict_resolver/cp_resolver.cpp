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

#include "rmf_scheduler/conflict_resolver/cp_resolver.hpp"

using CpSolverStatus = operations_research::sat::CpSolverStatus;
using CpSolverResponse = operations_research::sat::CpSolverResponse;
using SatParameters = operations_research::sat::SatParameters;
using Model = operations_research::sat::Model;

namespace rmf_scheduler
{
namespace conflict_resolver
{
CpResolver::CpResolver()
{
  // Do Nothing
}

std::shared_ptr<CpResolver> CpResolver::make()
{
  std::shared_ptr<CpResolver> cp_resolver(new CpResolver());

  return cp_resolver;
}

auto CpResolver::deconflict(
  const std::unordered_map<std::string, Event> & events,
  const uint64_t window_start, const uint64_t window_end, bool optimize, float time_limit)
-> std::unordered_map<std::string, EventChange>
{
  if (window_end <= window_start) {
    std::cout << "INVALID WINDOW GIVEN" << std::endl;
    return {};
  }
  // Creat model builder
  CpModelBuilder _model_builder;

  // Set time limit for solving
  Model model;
  SatParameters parameters;
  parameters.set_max_time_in_seconds(time_limit);
  model.Add(NewSatParameters(parameters));

  // initialise map that stores list of events according to robot
  // {robot, list of event ids}
  std::unordered_map<std::string, std::vector<EventVarPtr>> robots_tasks_map;

  // Robot to vector of interval map for settings constraints
  std::unordered_map<std::string, std::vector<IntervalVar>> robot_to_intervals_map;

  // initalise map that stores vector of events according to location
  std::unordered_map<std::string, std::vector<EventVarPtr>> location_tasks_map;

  // Location to vector of interval map for settings constraints
  std::unordered_map<std::string, std::vector<IntervalVar>> location_to_intervals_map;

  // initialise vector of flight schedule events
  std::vector<EventVarPtr> flight_events;

  // Create objective linear expression
  LinearExpr objective_func;

  // Create and Categorise the events
  for (auto event : events) {
    // Create EventVar struct
    EventVarPtr event_var = convert(event.second, window_start, window_end, _model_builder);

    if (optimize) {
      // Add difference in orignal start time and assigned start time to objective function
      add_abs_diff(
        window_start, window_end, event.second, event_var, _model_builder,
        objective_func);
    }

    // Add interval to location to intervals map
    location_to_intervals_map[event.second.location].push_back(event_var->interval);

    if (event.second.is_flight_schedule) {
      // Flight based events should not be able to be rescheduled
      _model_builder.AddEquality(event_var->start, event.second.start_time);

      // Add event to flight events vector
      flight_events.push_back(event_var);
      continue;
    }

    // Add event var to robots events map
    robots_tasks_map[event.second.robot].push_back(event_var);

    // Add interval to robot to intervals map
    robot_to_intervals_map[event.second.robot].push_back(event_var->interval);
  }

  // Ensure every task event that a robot takes should not overlap
  add_no_overlap(robot_to_intervals_map, _model_builder);

  // Ensure that events in the same location do not overlap
  add_no_overlap(location_to_intervals_map, _model_builder);

  if (optimize) {
    // Minimize the amount of rescheduling
    _model_builder.Minimize(objective_func);
  }

  // Solve the model
  CpSolverResponse response = SolveCpModel(_model_builder.Build(), &model);

  return get_changes(robots_tasks_map, events, response);
}

// Converts an Event into and EventVarPtr
auto CpResolver::convert(
  const Event & event, const uint64_t window_start,
  const uint64_t window_end, CpModelBuilder & _model_builder) -> EventVarPtr
{
  // Create variables
  Domain domain(window_start, window_end);
  IntVar start = _model_builder.NewIntVar(domain).WithName("start_" + event.id);
  IntVar end = _model_builder.NewIntVar(domain).WithName("end_" + event.id);
  IntervalVar interval = _model_builder.NewIntervalVar(start, event.duration, end)
    .WithName("interval_" + event.id);

  EventVarPtr event_var = std::make_shared<EventVar>();
  event_var->start = start;
  event_var->end = end;
  event_var->interval = interval;
  event_var->id = event.id;
  return event_var;
}

// Adds the difference in original start time and assigned start to objective function
auto CpResolver::add_abs_diff(
  const uint64_t window_start, const uint64_t window_end,
  const Event & event, const EventVarPtr event_var, CpModelBuilder & _model_builder,
  LinearExpr & objective_func) -> void
{
  // Calculate the absolute value of the difference in scheduled start time
  // and original start time
  Domain diff_domain(-int64_t(window_end - window_start), int64_t(window_end - window_start));
  IntVar negative_diff =
    _model_builder.NewIntVar(diff_domain).WithName("negative_diff_" + event_var->id);


  // let x = final start time - original start time
  // (x + y = 0)  == (x = -y)
  _model_builder.AddEquality((event_var->start - event.start_time) + negative_diff, 0);

  // abs_x
  IntVar abs_diff = _model_builder.NewIntVar({0, int64_t(window_end - window_start)})
    .WithName("abs_diff_" + event.id);

  // abs_x = max(x, y)
  // since y = -x
  // abs_x = max(x, -x)
  // therefore ==> abs_x = |x|
  _model_builder.AddMaxEquality(abs_diff, {(event_var->start - event.start_time), negative_diff});

  // Sum up all the change start times
  objective_func += abs_diff;
}

// Adds no overlaping constraints for a map of vector of IntervalVars
auto CpResolver::add_no_overlap(
  const std::unordered_map<std::string, std::vector<IntervalVar>> & intervals,
  CpModelBuilder & _model_builder) -> void
{
  for (auto interval : intervals) {
    _model_builder.AddNoOverlap(interval.second);
  }
}


// Get a map of vector of IntervalVars and returns the change
auto CpResolver::get_changes(
  const std::unordered_map<std::string, std::vector<EventVarPtr>> & robots_tasks_map,
  const std::unordered_map<std::string, Event> & events,
  CpSolverResponse & response) -> std::unordered_map<std::string, EventChange>
{
  if (response.status() == CpSolverStatus::OPTIMAL ||
    response.status() == CpSolverStatus::FEASIBLE)
  {
    std::cout << "Optimal Re-Scheduling Amount: " << response.objective_value() << std::endl;

    // initialise vector of event changes
    std::unordered_map<std::string, EventChange> event_changes;

    for (auto robot : robots_tasks_map) {
      for (int i = 0; i < robot.second.size(); i++) {
        EventVarPtr robot_event_var = robot.second[i];
        auto event_itr = events.find(robot_event_var->id);

        // TODO(KW): Consider if we even need this check.
        if (event_itr == events.end()) {
          std::cout << "Somehow event {" << robot_event_var->id << "does not exist" << std::endl;
          // handle issue
        }

        // Get original start time
        const uint64_t original_start_time = event_itr->second.start_time;

        // Get final start time
        const uint64_t final_start_time =
          operations_research::sat::SolutionIntegerValue(response, robot_event_var->start);

        // Check if event has been rescheduled
        if (original_start_time == final_start_time) {
          continue;
        }

        // Event has been reschedule, so we add it to the list of changed events
        event_changes.emplace(
          std::make_pair(
            robot_event_var->id,
            EventChange {
              robot_event_var->id,
              original_start_time,
              final_start_time,
              event_itr->second.duration
            }
          )
        );
      }
    }

    return event_changes;
  } else {
    std::cout << "Solution not FEASIBLE try deleting some events" << std::endl;
    return {};
  }
}


void CpResolver::clear()
{
  //
}

}  // namespace conflict_resolver
}  // namespace rmf_scheduler
