// Copyright 2024 ROS Industrial Consortium Asia Pacific
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

#include "rmf2_scheduler/data/event.hpp"

class TestDataEvent : public ::testing::Test
{
};

TEST_F(TestDataEvent, empty_create) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Event event;

  EXPECT_EQ(event.id, "");
  EXPECT_EQ(event.type, "");
  EXPECT_FALSE(event.description.has_value());
  EXPECT_TRUE(event.start_time == Time(0));
  EXPECT_FALSE(event.duration.has_value());
  EXPECT_FALSE(event.series_id.has_value());
  EXPECT_FALSE(event.process_id.has_value());
}

TEST_F(TestDataEvent, minimal_create) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Event event(
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "event_type",
    Time(1672625700, 0)
  );

  EXPECT_EQ(event.id, "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  EXPECT_EQ(event.type, "event_type");
  EXPECT_FALSE(event.description.has_value());
  EXPECT_TRUE(event.start_time == Time(1672625700, 0));
  EXPECT_FALSE(event.duration.has_value());
  EXPECT_FALSE(event.series_id.has_value());
  EXPECT_FALSE(event.process_id.has_value());
}

TEST_F(TestDataEvent, full_create) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Event event(
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "event_type",
    "This is a event!",
    Time(1672625700, 0),
    Duration(3600, 0),
    "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
    "13aa1c62-64ca-495d-a4b7-84de6a00f56a"
  );

  EXPECT_EQ(event.id, "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  EXPECT_EQ(event.type, "event_type");
  EXPECT_EQ(event.description, "This is a event!");
  EXPECT_TRUE(event.start_time == Time(1672625700, 0));
  EXPECT_TRUE(event.duration == Duration(3600, 0));
  EXPECT_EQ(event.series_id, "7c98b392-2131-4528-92d2-7e7f22d0a9a5");
  EXPECT_EQ(event.process_id, "13aa1c62-64ca-495d-a4b7-84de6a00f56a");
}

TEST_F(TestDataEvent, equal) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Event event(
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "event_type",
    "This is a event!",
    Time(1672625700, 0),
    Duration(3600, 0),
    "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
    "13aa1c62-64ca-495d-a4b7-84de6a00f56a"
  );

  {  // Equal
    Event event_to_compare(
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "event_type",
      "This is a event!",
      Time(1672625700, 0),
      Duration(3600, 0),
      "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "13aa1c62-64ca-495d-a4b7-84de6a00f56a"
    );

    EXPECT_EQ(event, event_to_compare);
  }

  {  // Different ID
    Event event_to_compare(
      "123456",
      "event_type",
      "This is a event!",
      Time(1672625700, 0),
      Duration(3600, 0),
      "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "13aa1c62-64ca-495d-a4b7-84de6a00f56a"
    );

    EXPECT_NE(event, event_to_compare);
  }
}
