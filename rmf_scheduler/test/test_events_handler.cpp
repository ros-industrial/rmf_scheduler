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

#include "gtest/gtest.h"
#include "rmf_scheduler/events_handler.hpp"
#include "rmf_scheduler/event_utils.hpp"

TEST(TestEventsHandler, BasicCRUD) {
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)
  EventsHandler events_handler;

  // Add one event
  Event event1 {
    "First Event for testing",  // description
    "robot_task",               // type
    0,                          // start time
    10,                         // end time
    "abcdefg12345",             // id
    "",                         // series id
    ""                          // dag id
  };

  events_handler.add_event(event1);

  // Check if event is added correctly
  EXPECT_TRUE(events_handler.has_event("abcdefg12345"));

  EXPECT_TRUE(
    utils::is_equal(
      event1,
      events_handler.get_event("abcdefg12345")
  ));

  const auto & all_events = events_handler.get_all_events();
  EXPECT_TRUE(
    utils::is_equal(
      event1,
      all_events.at("abcdefg12345")
  ));

  // Update added event
  Event event1_new {
    "First Event for testing modified",
    "robot_task",
    0,
    10,
    "abcdefg12345",  // id stays the same
    "dks",
    "aaa"
  };

  events_handler.update_event(event1_new);

  // Check if event is updated correctly
  EXPECT_TRUE(events_handler.has_event("abcdefg12345"));

  EXPECT_TRUE(
    utils::is_equal(
      event1_new,
      events_handler.get_event("abcdefg12345")
  ));

  const auto & new_all_events = events_handler.get_all_events();
  EXPECT_TRUE(
    utils::is_equal(
      event1_new,
      new_all_events.at("abcdefg12345")
  ));
}
