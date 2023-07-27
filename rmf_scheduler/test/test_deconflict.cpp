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
#include "rmf_scheduler/scheduler.hpp"
#include "rmf_scheduler/conflict_resolver/identifier.hpp"
#include "rmf_scheduler/conflict_resolver/cp_resolver.hpp"
#include "nlohmann/json.hpp"

#include "boost/uuid/uuid.hpp"
#include "boost/uuid/random_generator.hpp"
#include "boost/uuid/uuid_io.hpp"

namespace rmf_scheduler::conflict_resolver::test_utils
{
auto load_no_clashing_events() -> std::unordered_map<std::string, Event>;
auto load_clashing_events() -> std::unordered_map<std::string, Event>;
auto gen_uuid() -> std::string;
}

// Test that no conflict schedule does not get rescheduled
TEST(TestScheduler, NoConflictResolution)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  const uint64_t WINDOW_START = 0;
  const uint64_t WINDOW_END = 60 * 24;  // 24hr * 60 min/hr

  // Invalid JSON
  std::shared_ptr<conflict_resolver::CpResolver> cp_resolver =
    conflict_resolver::CpResolver::make();

  std::unordered_map<std::string, Event> events;

  std::unordered_map<std::string, conflict_resolver::EventChange> changed_events;

  // Load a schedule of events that do not clash
  events = conflict_resolver::test_utils::load_no_clashing_events();

  // There are no clashes in events so there should be no changes made
  changed_events =
    cp_resolver->deconflict(events, WINDOW_START, WINDOW_END, true);
  EXPECT_EQ(changed_events.size(), 0);
}

// Test that conflicting scheduled events get resolved
TEST(TestScheduler, ConflictResolution)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  const uint64_t WINDOW_START = 0;
  const uint64_t WINDOW_END = 60 * 24;  // 24hr * 60 min/hr

  std::shared_ptr<conflict_resolver::CpResolver> cp_resolver =
    conflict_resolver::CpResolver::make();

  // Load a schedule of events that do not clash
  std::unordered_map<std::string, Event> events =
    conflict_resolver::test_utils::load_clashing_events();

  conflict_resolver::Identifier identifier;
  std::vector<std::pair<std::string, std::string>> initial_conflicts;
  initial_conflicts = identifier.identify_conflicts(events);

  // There should be conflicts initially
  EXPECT_NE(initial_conflicts.size(), 0);

  // There are no clashes in events so there should be no changes made
  std::unordered_map<std::string, conflict_resolver::EventChange> changed_events =
    cp_resolver->deconflict(events, WINDOW_START, WINDOW_END, true);

  for (auto change : changed_events) {
    auto event_itr = events.find(change.first);
    if (event_itr == events.end()) {
      std::cout << "Event: " << change.first << "not found!" << std::endl;
    }
    // Update changed events
    event_itr->second.start_time = change.second.final_start_time;
  }

  // There should be conflicts in the end
  std::vector<std::pair<std::string, std::string>> final_conflicts;
  final_conflicts = identifier.identify_conflicts(events);
  EXPECT_EQ(final_conflicts.size(), 0);
}


////////////////////// TEST UTIL FUNCTIONS ///////////////
namespace rmf_scheduler::conflict_resolver::test_utils
{

// std::string gen_uuid()
// {
//   boost::uuids::uuid uuid = boost::uuids::random_generator()();
//   return boost::uuids::to_string(uuid);
// }

// Load a set of robot events that do not clash
std::unordered_map<std::string, Event> load_clashing_events()
{
  std::unordered_map<std::string, Event> events;
  const std::vector<std::string> locations = {
    "location_1",
    "location_2",
    "location_3",
    "location_4",
    "location_5",
    "area_54"
  };
  const uint64_t start = 0;
  const uint64_t duration = 60;

  // Add robot task events
  for (int i = 0; i < 5; i++) {  // 10 robots
    const std::string robot = "robot_" + std::to_string(i);
    for (int j = 0; j < 10; j++) {  // 10 events
      std::string id = gen_uuid();
      events.emplace(
        std::make_pair(
          id,
          Event{
          "",                               // description
          "default/robot_task",             // type
          start + (duration / 2) * j,       // start time
          duration,                         // duration
          id,                               // id
          "",                               // series id
          "",                               // dag id
          false,                            // flight schedule
          robot,                            // robot
          locations[j % locations.size()]   // random location so no clashes
        }
        )
      );
    }
  }

  // Add flight schedule based events
  for (int i = 0; i < 10; i++) {  // 10 events
    std::string id = gen_uuid();
    events.emplace(
      std::make_pair(
        id,
        Event{
        "",                               // description
        "flight-schedule",                // type
        start + 60 * i,                   // start time
        duration,                         // duration
        id,                               // id
        "",                               // series id
        "",                               // dag id
        true,                             // flight schedule
        "",                               // robot
        locations[i % locations.size()]   // random location so no clashes
      }
      )
    );
  }
  return events;
}

std::unordered_map<std::string, Event> load_no_clashing_events()
{
  std::unordered_map<std::string, Event> events;
  const uint64_t start = 0;
  const uint64_t duration = 60;
  for (int i = 0; i < 2; i++) {   // 10 robots
    const std::string robot = "robot_" + std::to_string(i);
    for (int j = 0; j < 3; j++) {  // 10 events
      std::string id = gen_uuid();
      events.emplace(
        std::make_pair(
          id,
          Event{
          "",                       // description
          "default/robot_task",     // type
          start + duration * j,     // start time
          duration,                 // duration
          id,                       // id
          "",                       // series id
          "",                       // dag id
          false,                    // flight schedule
          robot,                    // robot
          gen_uuid()                // random location so no clashes
        }
        )
      );
    }
  }
  return events;
}

void print_event_map(const std::unordered_map<std::string, Event> & events)
{
  for (auto event : events) {
    std::cout << "Event \n"
              << std::endl
              << "\tid: "
              << event.second.id
              << "\n \tstart time: "
              << event.second.start_time
              << "\n \tduration: "
              << event.second.duration
              << "\n \tis flight schedule: "
              << event.second.is_flight_schedule
              << "\n \trobot: "
              << event.second.robot
              << "\n \tlocation: "
              << event.second.location
              << std::endl;
  }
}

}  // namespace rmf_scheduler::conflict_resolver::test_utils
