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

#include <gtest/gtest.h>

#include <cstdint>

#include <rmf_scheduler/conflict_resolver/identifier.hpp>

#include "boost/uuid/uuid.hpp"
#include "boost/uuid/random_generator.hpp"
#include "boost/uuid/uuid_io.hpp"

namespace rmf_scheduler::conflict_resolver::test_utils
{
auto load_events()
->std::unordered_map<std::string, rmf_scheduler::Event>;
}  // namespace rmf_scheduler::conflict_resolver::test_utils

/// No Conflict
/**
 * For Example
 * Schedule A : 10 - 15
 * Schedule B : 25 - 30
*/
TEST(TestIdentifier, NoConflict)
{
  using Identifier = rmf_scheduler::conflict_resolver::Identifier;

  Identifier identifier;
  uint64_t result = 0;

  uint64_t duration = identifier.conflict_check(10, 15, 25, 30);
  EXPECT_EQ(duration, result);
}

/// Conflict Type A
/**
 * For Example
 * Schedule A : 10 - 15
 * Schedule B : 14 - 18
*/
TEST(TestIdentifier, ConflictTypeA)
{
  using Identifier = rmf_scheduler::conflict_resolver::Identifier;

  Identifier identifier;
  uint64_t result = 2;

  uint64_t duration = identifier.conflict_check(10, 15, 14, 18);
  EXPECT_EQ(duration, result);
}

/// Conflict Type B
/**
 * For Example
 * Schedule A : 10 - 15
 * Schedule B : 7  - 11
*/
TEST(TestIdentifier, ConflictTypeB)
{
  using Identifier = rmf_scheduler::conflict_resolver::Identifier;

  Identifier identifier;
  uint64_t result = 9;

  uint64_t duration = identifier.conflict_check(10, 15, 7, 11);
  EXPECT_EQ(duration, result);
}

/// Conflict Type C
/**
 * For Example
 * Schedule A : 10 - 15
 * Schedule B : 11 - 14
*/
TEST(TestIdentifier, ConflictTypeC)
{
  using Identifier = rmf_scheduler::conflict_resolver::Identifier;

  Identifier identifier;
  uint64_t result = 5;

  uint64_t duration = identifier.conflict_check(10, 15, 11, 14);
  EXPECT_EQ(duration, result);
}

/// Conflict Type D
/**
 * For Example
 * Schedule A : 10 - 15
 * Schedule B : 5  - 21
*/
TEST(TestIdentifier, ConflictTypeD)
{
  using Identifier = rmf_scheduler::conflict_resolver::Identifier;

  Identifier identifier;
  uint64_t result = 11;

  uint64_t duration = identifier.conflict_check(10, 15, 5, 21);
  EXPECT_EQ(duration, result);
}

TEST(TestIdentifier, RobotCategoriser)
{
  using Identifier = rmf_scheduler::conflict_resolver::Identifier;
  using Event = rmf_scheduler::Event;

  std::unordered_map<std::string, Event> events =
    rmf_scheduler::conflict_resolver::test_utils::load_events();

  Identifier identifier;

  std::unordered_map<std::string, std::unordered_map<std::string, Event>>
  events_by_robots = identifier.categorise_by_robots(events);
  EXPECT_EQ(events_by_robots.size(), 5);
}

TEST(TestIdentifier, LocationCategoriser)
{
  using Identifier = rmf_scheduler::conflict_resolver::Identifier;
  using Event = rmf_scheduler::Event;

  std::unordered_map<std::string, Event> events;
  events = rmf_scheduler::conflict_resolver::test_utils::load_events();

  Identifier identifier;

  std::unordered_map<
    std::string, std::unordered_map<std::string, Event>> events_by_location;
  events_by_location = identifier.categorise_by_location(events);
  EXPECT_EQ(events_by_location.size(), 6);
}

// Test Utility Functions
namespace rmf_scheduler::conflict_resolver::test_utils
{

std::string gen_uuid()
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  return boost::uuids::to_string(uuid);
}

std::unordered_map<std::string, Event> load_events()
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

  // Create task events for robots;
  for (int i = 0; i < 5; i++) {  // 5 robots
    const std::string robot = "robot_" + std::to_string(i);
    for (int j = 0; j < 10; j++) {  // 10 events
      std::string id = gen_uuid();
      events.emplace(
        std::make_pair(
          id,
          Event{
          "",                               // description
          "default/robot_task",             // type
          start + duration * j,             // start time
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

  // Add flight schedule
  for (int i = 0; i < locations.size(); i++) {
    std::string id = gen_uuid();
    events.emplace(
      std::make_pair(
        id,
        Event{
        "",                               // description
        "flight_schedule",                // type
        start + duration * i,             // start time
        duration,                         // duration
        id,                               // id
        "",                               // series id
        "",                               // dag id
        true,                             // flight schedule
        "",                               // robot
        locations[i % locations.size()]   // add location
      }
      )
    );
  }
  return events;
}

}  // namespace rmf_scheduler::conflict_resolver::test_utils
