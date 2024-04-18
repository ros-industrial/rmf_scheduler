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

#include "ortools/base/logging.h"
#include "ortools/sat/cp_model.h"
#include "ortools/sat/cp_model.pb.h"
#include "ortools/sat/cp_model_solver.h"

#include "rmf_scheduler/conflict/cp_solver.hpp"
#include "rmf_scheduler/log.hpp"

namespace rmf_scheduler
{
namespace conflict
{

using CpModelBuilder = operations_research::sat::CpModelBuilder;
using CpSolverResponse = operations_research::sat::CpSolverResponse;
using IntVar = operations_research::sat::IntVar;
using IntervalVar = operations_research::sat::IntervalVar;
using Domain = operations_research::Domain;
using LinearExpr = operations_research::sat::LinearExpr;
using CpSolverStatus = operations_research::sat::CpSolverStatus;
using CpSolverResponse = operations_research::sat::CpSolverResponse;
using SatParameters = operations_research::sat::SatParameters;
using Model = operations_research::sat::Model;

class CpSolver::Impl
{
public:
  struct EventVar
  {
    IntVar start;
    IntervalVar interval;
  };

  using EventVarPtr = std::shared_ptr<EventVar>;

  struct EventCache
  {
    explicit EventCache(const data::Event & _event)
    {
      event = _event;
    }

    data::Event event;
    EventVarPtr var;
  };

  // Linear problem
  struct Problem
  {
    // Objective function
    LinearExpr objective_func;

    // Objective variable
    std::vector<IntVar> objective_vars;

    // Creat model builder
    CpModelBuilder model_builder;
  };

  std::unordered_map<std::string, EventCache> cache;

  std::shared_ptr<Problem> p;

  // Configuration
  // Include time limit for solving
  Model model;
  Window window;

  /// Converts an Event into and EventVarPtr
  /**
   * \param[in] event event to get converted
   * \return[out] EventVar shared pointer
   */
  EventVarPtr new_var(const data::Event & event);

  // Adds the difference in original start time and assigned start to objective function
  void add_abs_diff(const data::Event & event, const EventVarPtr event_var);

  /// Output event changes
  /**
   * \return[out] event changes
   */
  std::vector<EventChange> interpret_response(CpSolverResponse & response);
};

CpSolver::CpSolver()
: impl_(std::make_unique<Impl>())
{
  // Do Nothing
}

CpSolver::~CpSolver()
{
  // Do Nothing
}

std::shared_ptr<CpSolver> CpSolver::make()
{
  std::shared_ptr<CpSolver> cp_resolver(new CpSolver());

  return cp_resolver;
}

void CpSolver::init(
  const std::vector<data::Event> & events,
  const Window & window,
  double time_limit)
{
  if (window.end_time <= window.start_time) {
    throw std::invalid_argument("INVALID WINDOW GIVEN");
  }

  // Events
  for (auto & event : events) {
    impl_->cache.emplace(event.id, event);
  }

  // Set time limit
  SatParameters parameters;
  if (time_limit > 0) {
    parameters.set_max_time_in_seconds(time_limit);
  }
  impl_->model.Add(NewSatParameters(parameters));

  // Initialize Optimization Window
  impl_->window = window;

  // Construct new problem
  impl_->p = std::make_shared<Impl::Problem>();
}

std::vector<EventChange> CpSolver::solve(bool optimize)
{
  if (optimize) {
    // Minimize the amount of rescheduling
    impl_->p->model_builder.Minimize(impl_->p->objective_func);
  }

  // Solve the model
  CpSolverResponse response = SolveCpModel(
    impl_->p->model_builder.Build(), &impl_->model);

  return impl_->interpret_response(response);
}


void CpSolver::add_objective(
  const std::vector<std::string> & ids,
  double /*priority*/)
{
  for (auto & id : ids) {
    auto itr = impl_->cache.find(id);
    if (itr == impl_->cache.end()) {
      continue;
    }

    // Initiate variable if not initialized yet
    if (!itr->second.var) {
      itr->second.var = impl_->new_var(itr->second.event);
    }

    impl_->add_abs_diff(itr->second.event, itr->second.var);
    impl_->p->objective_vars.push_back(itr->second.var->start);
  }
}


// Adds no overlaping constraints for a map of vector of IntervalVars
void CpSolver::add_no_overlap(const std::vector<std::string> & ids)
{
  std::vector<IntervalVar> intervals;
  for (auto & id : ids) {
    auto itr = impl_->cache.find(id);
    if (itr == impl_->cache.end()) {
      continue;
    }

    // Initiate variable if not initialized yet
    if (!itr->second.var) {
      itr->second.var = impl_->new_var(itr->second.event);
    }

    intervals.push_back(itr->second.var->interval);
  }
  impl_->p->model_builder.AddNoOverlap(intervals);
}


void CpSolver::mark_fixed(
  const std::vector<std::string> & ids)
{
  for (auto & id : ids) {
    auto itr = impl_->cache.find(id);
    if (itr == impl_->cache.end()) {
      continue;
    }

    // Initiate variable if not initialized yet
    if (!itr->second.var) {
      itr->second.var = impl_->new_var(itr->second.event);
    }

    impl_->p->model_builder.FixVariable(
      itr->second.var->start,
      static_cast<int64_t>(itr->second.event.start_time));
  }
}

// Converts an Event into and EventVarPtr
CpSolver::Impl::EventVarPtr CpSolver::Impl::new_var(const data::Event & event)
{
  // Create variables
  Domain domain(window.start_time, window.end_time);
  IntVar start = p->model_builder.NewIntVar(domain).WithName(event.id);
  IntervalVar interval = p->model_builder.NewFixedSizeIntervalVar(start, event.duration)
    .WithName("interval_" + event.id);

  EventVarPtr event_var = std::make_shared<EventVar>();
  event_var->start = start;
  event_var->interval = interval;
  return event_var;
}


// Adds the difference in original start time and assigned start to objective function
void CpSolver::Impl::add_abs_diff(
  const data::Event & event, const EventVarPtr event_var)
{
  // Calculate the absolute value of the difference in scheduled start time
  // and original start time
  Domain diff_domain(
    -int64_t(window.end_time - window.start_time),
    int64_t(window.end_time - window.start_time));
  IntVar negative_diff =
    p->model_builder.NewIntVar(diff_domain).WithName("negative_diff_" + event.id);


  // let x = final start time - original start time
  // (x + y = 0)  == (x = -y)
  p->model_builder.AddEquality((event_var->start - event.start_time) + negative_diff, 0);

  // abs_x
  IntVar abs_diff = p->model_builder.NewIntVar({0, int64_t(window.end_time - window.start_time)})
    .WithName("abs_diff_" + event.id);

  // abs_x = max(x, y)
  // since y = -x
  // abs_x = max(x, -x)
  // therefore ==> abs_x = |x|
  p->model_builder.AddMaxEquality(abs_diff, {(event_var->start - event.start_time), negative_diff});

  // Sum up all the change start times
  // TODO(anyone): add priority
  p->objective_func += abs_diff;
}


// Get a map of vector of IntervalVars and returns the change
std::vector<EventChange> CpSolver::Impl::interpret_response(CpSolverResponse & response)
{
  if (response.status() == CpSolverStatus::OPTIMAL ||
    response.status() == CpSolverStatus::FEASIBLE)
  {
    RS_LOG_INFO("Optimal Re-Scheduling Amount: ", response.objective_value());

    // initialise vector of event changes
    std::vector<EventChange> event_changes;

    for (auto var : p->objective_vars) {
      std::string id = var.Name();

      // Get original start time
      const uint64_t original_start_time = cache.at(id).event.start_time;

      // Get final start time
      const uint64_t final_start_time =
        operations_research::sat::SolutionIntegerValue(response, var);

      // Check if event has been rescheduled
      if (original_start_time == final_start_time) {
        continue;
      }

      // Event has been reschedule, so we add it to the list of changed events
      event_changes.push_back(
        EventChange {
            id,
            original_start_time,
            final_start_time,
          }
      );
    }

    return event_changes;
  } else {
    RS_LOG_WARN("Solution not FEASIBLE try deleting some events.");
    return {};
  }
}


void CpSolver::clear()
{
  impl_->cache.clear();
  impl_->p.reset();
}

}  // namespace conflict
}  // namespace rmf_scheduler
