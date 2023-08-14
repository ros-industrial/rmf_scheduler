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

#include <fstream>
#include <sstream>

#include "gtest/gtest.h"

#include "rmf_scheduler/test_utils.hpp"
#include "rmf_scheduler/sanitizer_macro.hpp"
#include "rmf_scheduler/conflict_identifier.hpp"
#include "rmf_scheduler/conflict_resolver/cp_solver.hpp"

// TODO(anyone): fix sanitizer failure
#ifndef __SANITIZE_ADDRESS__
#ifndef __SANITIZE_THREAD__

// Test that no conflict schedule does not get rescheduled
TEST(TestScheduler, NoConflictResolution)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  const uint64_t WINDOW_START = 0;
  const uint64_t WINDOW_END = 60 * 24;  // 24hr * 60 min/hr

  std::shared_ptr<conflict_resolver::CpSolver> cp_solver =
    conflict_resolver::CpSolver::make();

  // Load a schedule of events that do not clash
  std::vector<Event> events;
  events = test_utils::load_no_clashing_events(2, 3);

  // Initialize the solver
  cp_solver->init(events, {WINDOW_START, WINDOW_END});

  // Categorise the events by type
  auto event_by_type = utils::categorise_by_type(events);

  // Mark flight schedule as fixed
  auto flight_event_itr = event_by_type.find("flight-schedule");
  if (flight_event_itr != event_by_type.end()) {
    cp_solver->mark_fixed(flight_event_itr->second);
  }

  // Add robot task to the objective function
  auto robot_event_itr = event_by_type.find("default/robot_task");
  if (robot_event_itr != event_by_type.end()) {
    cp_solver->add_objective(robot_event_itr->second);
  }

  // No overlap for robot and zone
  auto event_by_filter = utils::categorise_by_filter(events, {"request::robot", "zone"});

  for (auto itr : event_by_filter) {
    for (auto itr2 : itr) {
      cp_solver->add_no_overlap(itr2.second);
    }
  }

  // There are no clashes in events so there should be no changes made
  auto result = cp_solver->solve(true);
  EXPECT_TRUE(result.empty());
}

// Test that conflicting scheduled events get resolved
TEST(TestScheduler, ConflictResolution)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  const uint64_t WINDOW_START = 0;
  const uint64_t WINDOW_END = 60 * 24;  // 24hr * 60 min/hr

  std::shared_ptr<conflict_resolver::CpSolver> cp_solver =
    conflict_resolver::CpSolver::make();

  // Load a schedule of events that clashes
  std::vector<Event> events = test_utils::load_clashing_events(5, 6, 3);

  auto initial_conflicts = utils::identify_conflicts(
    events, {}, {"request::robot", "zone"});

  // There should be conflicts initially
  EXPECT_FALSE(initial_conflicts.empty());

  // Initialize the solver
  cp_solver->init(events, {WINDOW_START, WINDOW_END}, 5.0);

  // Categorise the events by type
  auto event_by_type = utils::categorise_by_type(events);

  // Mark flight schedule as fixed
  auto flight_event_itr = event_by_type.find("flight-schedule");
  if (flight_event_itr != event_by_type.end()) {
    cp_solver->mark_fixed(flight_event_itr->second);
  }

  // Add robot task to the objective function
  auto robot_event_itr = event_by_type.find("default/robot_task");
  if (robot_event_itr != event_by_type.end()) {
    cp_solver->add_objective(robot_event_itr->second);
  }

  // Solve for a feasible solution
  auto event_by_filter = utils::categorise_by_filter(events, {"request::robot", "zone"});

  for (auto itr : event_by_filter) {
    for (auto itr2 : itr) {
      cp_solver->add_no_overlap(itr2.second);
    }
  }

  // Solve for a way to avoid conflict
  auto result = cp_solver->solve(true);
  EXPECT_FALSE(result.empty());

  // Implement the changes
  std::unordered_map<std::string, size_t> name_lookup;
  for (size_t i = 0; i < events.size(); i++) {
    name_lookup[events[i].id] = i;
  }

  for (auto & change : result) {
    std::cout << "Event id: " << change.id
              << "\n orig:" << change.original_start_time
              << ", final:" << change.final_start_time << std::endl;
    auto event_idx_itr = name_lookup.find(change.id);
    if (event_idx_itr == name_lookup.end()) {
      std::cout << "Event: " << change.id << " not found!" << std::endl;
    }

    events[event_idx_itr->second].start_time = change.final_start_time;
  }

  // There should not be any conflicts in the end
  auto final_conflicts = utils::identify_conflicts(
    events, {}, {"request::robot", "zone"});
  EXPECT_TRUE(final_conflicts.empty());
}

#endif
#endif
