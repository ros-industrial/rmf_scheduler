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

#ifndef RMF_SCHEDULER__CONFLICT_RESOLVER__CP_RESOLVER_HPP_
#define RMF_SCHEDULER__CONFLICT_RESOLVER__CP_RESOLVER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>

#include "ortools/base/logging.h"
#include "ortools/sat/cp_model.h"
#include "ortools/sat/cp_model.pb.h"
#include "ortools/sat/cp_model_solver.h"

#include "rmf_scheduler/event.hpp"

using CpModelBuilder = operations_research::sat::CpModelBuilder;
using CpSolverResponse = operations_research::sat::CpSolverResponse;
using IntVar = operations_research::sat::IntVar;
using IntervalVar = operations_research::sat::IntervalVar;
using Domain = operations_research::Domain;
using LinearExpr = operations_research::sat::LinearExpr;
using Event = rmf_scheduler::Event;

namespace rmf_scheduler
{
namespace conflict_resolver
{

struct EventVar
{
  IntVar start;
  IntVar end;
  IntervalVar interval;
  std::string id;
};

// Class to reflect changes of an event
struct EventChange
{
  std::string id;
  uint64_t original_start_time;
  uint64_t final_start_time;
  uint64_t duration;
};

using EventVarPtr = std::shared_ptr<EventVar>;

class CpResolver
{
public:
  CpResolver();

  static std::shared_ptr<CpResolver> make();

  /// Return Event Changes required to deconflict current schedule with
  /// requested changes
  /**
   * \param[in] events - List of events to deconflict
   * \param[in] lhs_end_time - window of time to reschedule for
   * \return[out] List of rescheduled events
   */
  auto deconflict(
    const std::unordered_map<std::string, Event> & events,
    const uint64_t window_start, const uint64_t window_end,
    bool optimize, float time_limit = 5.0) -> std::unordered_map<std::string, EventChange>;

  void clear();

private:
  /// Converts an Event into and EventVarPtr
  /**
   * \param[in] events - List of events to deconflict
   * \param[in] window_start - List of events to deconflict
   * \param[in] window_end - window of time to reschedule for
   * \param[in] _model_builder - model builder class
   * \return[out] EventVar shared pointer
   */
  auto convert(
    const Event & event, const uint64_t window_start,
    const uint64_t window_end, CpModelBuilder & _model_builder) -> EventVarPtr;

  // Adds the difference in original start time and assigned start to objective function
  /**
   * \param[in] window_start - List of events to deconflict
   * \param[in] window_end - window of time to reschedule for
   * \param[in] _model_builder - model builder class
   * \param[in] objective_func - objective function
   * \return[out] void - void
   */
  auto add_abs_diff(
    const uint64_t window_start, const uint64_t window_end,
    const Event & event, const EventVarPtr event_var, CpModelBuilder & _model_builder,
    LinearExpr & objective_func) -> void;

  /// Adds no overlaping constraints for a map of vector of IntervalVars
  /**
   * \param[in] intervals - an unordered map of vectors of intervals to be constrained
   * \param[in] _model_class - model builder class
   * \return[out] void
   */
  auto add_no_overlap(
    const std::unordered_map<std::string, std::vector<IntervalVar>> & intervals_map,
    CpModelBuilder & _model_builder) -> void;

  // Input a map of vector of IntervalVars and returns the changes in events
  /**
   * \param[in] robots_tasks_maps - an unordered map of vectors of intervals for robot tasks
   * \param[in] events - events
   * \return[out] unordered map of EventChange which reflects event changes
   */
  auto get_changes(
    const std::unordered_map<std::string, std::vector<EventVarPtr>> & robots_tasks_map,
    const std::unordered_map<std::string, Event> & events,
    CpSolverResponse & response) -> std::unordered_map<std::string, EventChange>;
};

}  // namespace conflict_resolver
}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__CONFLICT_RESOLVER__CP_RESOLVER_HPP_
