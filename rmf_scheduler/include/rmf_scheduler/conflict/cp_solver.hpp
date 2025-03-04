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

#ifndef RMF_SCHEDULER__CONFLICT__CP_SOLVER_HPP_
#define RMF_SCHEDULER__CONFLICT__CP_SOLVER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "rmf_scheduler/data/event.hpp"
#include "rmf_scheduler/window.hpp"

namespace rmf_scheduler
{
namespace conflict
{

// Class to reflect changes of an event
struct EventChange
{
  std::string id;
  uint64_t original_start_time;
  uint64_t final_start_time;
};

class CpSolver
{
public:
  class Impl;

  /// Initiate the solver with a list of events
  /**
   * \param[in] events List of events to be used by the solver
   */
  static std::shared_ptr<CpSolver> make();

  virtual ~CpSolver();

  /// Initialize the Solver with events of interest
  void init(
    const std::vector<data::Event> & events,
    const Window & window,
    double time_limit = -1.0);

  /// Select event as optimization objective
  /**
   * \param[in] ids Selected event ids for the objective function
   * \param[in] priority priority in the objective function
   */
  void add_objective(
    const std::vector<std::string> & ids,
    double priority = 1.0);

  /// Adds no overlaping constraints based on selected events
  /**
   * \param[in] ids Selected event ids for no overlap constraints
   */
  void add_no_overlap(const std::vector<std::string> & ids);

  /// Mark event as fixed event
  /**
   * \param[in] ids Selected event ids to mark as fixed event.
   */
  void mark_fixed(
    const std::vector<std::string> & ids);

  /// Return Event Changes required to deconflict current schedule with
  /// requested changes
  /**
   * \return[out] List of rescheduled events
   */
  std::vector<EventChange> solve(bool optimize);

  /// Reset the solver
  void clear();

private:
  CpSolver();

  std::unique_ptr<Impl> impl_;
};

}  // namespace conflict
}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__CONFLICT__CP_SOLVER_HPP_
