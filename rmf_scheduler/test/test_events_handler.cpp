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
#include "rmf_scheduler/data/events_handler.hpp"
#include "rmf_scheduler/test_utils.hpp"

TEST(TestEventsHandler, BasicCRUD) {
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)
  data::EventsHandler eh;

  // Add one event
  std::string event_id = "abcdefg12345";

  data::Event event1 {
    "First Event for testing",  // description
    "robot_task",               // type
    0,                          // start time
    10,                         // duration
    event_id,                   // id
    "",                         // series id
    "",                         // dag id
    ""                          // event details
  };

  eh.add_event(event1);

  // Check if event is added correctly
  EXPECT_TRUE(eh.has_event(event_id));

  EXPECT_TRUE(
    test_utils::is_event_equal(
      event1,
      eh.get_event(event_id)
  ));

  const auto & all_events = eh.get_all_events();
  EXPECT_TRUE(
    test_utils::is_event_equal(
      event1,
      all_events.at(event_id)
  ));

  // Update added event
  data::Event event1_new {
    "First Event for testing modified",
    "robot_task",
    0,
    10,
    event_id,  // id stays the same
    "dks",
    "aaa",
    ""
  };

  eh.update_event(event1_new);

  // Check if event is updated correctly
  EXPECT_TRUE(eh.has_event(event_id));

  EXPECT_TRUE(
    test_utils::is_event_equal(
      event1_new,
      eh.get_event(event_id)
  ));

  const auto & new_all_events = eh.get_all_events();
  EXPECT_TRUE(
    test_utils::is_event_equal(
      event1_new,
      new_all_events.at(event_id)
  ));

  // Delete added event
  eh.delete_event(event_id);

  // Check if event is deleted
  EXPECT_FALSE(eh.has_event(event_id));

  const auto & empty_all_events = eh.get_all_events();
  EXPECT_TRUE(empty_all_events.empty());
}

TEST(TestEventsHandler, StartTimeLookupBasic)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Generate randomized events
  auto events_vector = test_utils::random_event_generator(
    0,
    100,
    1000);

  data::EventsHandler eh;

  for (auto & event : events_vector) {
    eh.add_event(event);
  }

  // Test lookup events
  // Compare results between linear approach vs hash multi map approach
  uint64_t lower_bound = 20;
  uint64_t upper_bound = 44;
  auto result_eh = eh.lookup_events(lower_bound, upper_bound);
  std::vector<data::Event> result_greedy;
  for (auto & event : events_vector) {
    if (event.start_time >= lower_bound && event.start_time <= upper_bound) {
      result_greedy.push_back(event);
    }
  }

  EXPECT_TRUE(
    test_utils::is_event_vector_equal(
      result_eh,
      result_greedy
    )
  );
}
