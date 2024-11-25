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

#include "rmf2_scheduler/data/task.hpp"

class TestDataTask : public ::testing::Test
{
};

TEST_F(TestDataTask, empty_create) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Task task;

  EXPECT_EQ(task.id, "");
  EXPECT_EQ(task.type, "");
  EXPECT_FALSE(task.description.has_value());
  EXPECT_TRUE(task.start_time == Time(0));
  EXPECT_FALSE(task.duration.has_value());
  EXPECT_FALSE(task.series_id.has_value());
  EXPECT_FALSE(task.process_id.has_value());
  EXPECT_FALSE(task.resource_id.has_value());
  EXPECT_FALSE(task.deadline.has_value());
  EXPECT_EQ(task.status, "");
  EXPECT_FALSE(task.planned_start_time.has_value());
  EXPECT_FALSE(task.planned_duration.has_value());
  EXPECT_FALSE(task.estimated_duration.has_value());
  EXPECT_FALSE(task.actual_start_time.has_value());
  EXPECT_FALSE(task.actual_duration.has_value());
  EXPECT_TRUE(task.task_details.is_null());
}

TEST_F(TestDataTask, minimal_create) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // from scratch
    Task task(
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "task_type",
      Time(1672625700, 0),
      "draft"
    );

    EXPECT_EQ(task.id, "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
    EXPECT_EQ(task.type, "task_type");
    EXPECT_FALSE(task.description.has_value());
    EXPECT_TRUE(task.start_time == Time(1672625700, 0));
    EXPECT_FALSE(task.duration.has_value());
    EXPECT_FALSE(task.series_id.has_value());
    EXPECT_FALSE(task.process_id.has_value());
    EXPECT_FALSE(task.resource_id.has_value());
    EXPECT_FALSE(task.deadline.has_value());
    EXPECT_EQ(task.status, "draft");
    EXPECT_FALSE(task.planned_start_time.has_value());
    EXPECT_FALSE(task.planned_duration.has_value());
    EXPECT_FALSE(task.estimated_duration.has_value());
    EXPECT_FALSE(task.actual_start_time.has_value());
    EXPECT_FALSE(task.actual_duration.has_value());
    EXPECT_TRUE(task.task_details.is_null());
  }

  {  // from event
    Task task(
      Event(
        "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
        "task_type",
        Time(1672625700, 0)
      ),
      "draft"
    );

    EXPECT_EQ(task.id, "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
    EXPECT_EQ(task.type, "task_type");
    EXPECT_FALSE(task.description.has_value());
    EXPECT_TRUE(task.start_time == Time(1672625700, 0));
    EXPECT_FALSE(task.duration.has_value());
    EXPECT_FALSE(task.series_id.has_value());
    EXPECT_FALSE(task.process_id.has_value());
    EXPECT_FALSE(task.resource_id.has_value());
    EXPECT_FALSE(task.deadline.has_value());
    EXPECT_EQ(task.status, "draft");
    EXPECT_FALSE(task.planned_start_time.has_value());
    EXPECT_FALSE(task.planned_duration.has_value());
    EXPECT_FALSE(task.estimated_duration.has_value());
    EXPECT_FALSE(task.actual_start_time.has_value());
    EXPECT_FALSE(task.actual_duration.has_value());
    EXPECT_TRUE(task.task_details.is_null());
  }
}

TEST_F(TestDataTask, full_create) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // from scratch
    Task task(
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "task_type",
      "This is a task!",
      Time(1672625700, 0),
      Duration(3600, 0),
      "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
      "09d5323d-43e0-4668-b53a-3cf33f9b9a96",
      Time(1672632900, 0),
      "ongoing",
      Time(1672625700, 0),
      Duration(3600, 0),
      Duration(3700, 0),
      Time(1672625760, 0),
      Duration(3760, 0),
      R"({"some_field": "some_value"})"_json
    );

    EXPECT_EQ(task.id, "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
    EXPECT_EQ(task.type, "task_type");
    EXPECT_EQ(task.description, "This is a task!");
    EXPECT_TRUE(task.start_time == Time(1672625700, 0));
    EXPECT_TRUE(task.duration == Duration(3600, 0));
    EXPECT_EQ(task.series_id, "7c98b392-2131-4528-92d2-7e7f22d0a9a5");
    EXPECT_EQ(task.process_id, "13aa1c62-64ca-495d-a4b7-84de6a00f56a");
    EXPECT_EQ(task.resource_id, "09d5323d-43e0-4668-b53a-3cf33f9b9a96");
    EXPECT_TRUE(task.deadline == Time(1672632900, 0));
    EXPECT_EQ(task.status, "ongoing");
    EXPECT_TRUE(task.planned_start_time == Time(1672625700, 0));
    EXPECT_TRUE(task.planned_duration == Duration(3600, 0));
    EXPECT_TRUE(task.estimated_duration == Duration(3700, 0));
    EXPECT_TRUE(task.actual_start_time == Time(1672625760, 0));
    EXPECT_TRUE(task.actual_duration == Duration(3760, 0));
    EXPECT_EQ(task.task_details, R"({"some_field": "some_value"})"_json);
  }

  {  // from event
    Task task(
      Event(
        "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
        "task_type",
        "This is a task!",
        Time(1672625700, 0),
        Duration(3600, 0),
        "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
        "13aa1c62-64ca-495d-a4b7-84de6a00f56a"
      ),
      "09d5323d-43e0-4668-b53a-3cf33f9b9a96",
      Time(1672632900, 0),
      "ongoing",
      Time(1672625700, 0),
      Duration(3600, 0),
      Duration(3700, 0),
      Time(1672625760, 0),
      Duration(3760, 0),
      R"({"some_field": "some_value"})"_json
    );

    EXPECT_EQ(task.id, "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
    EXPECT_EQ(task.type, "task_type");
    EXPECT_EQ(task.description, "This is a task!");
    EXPECT_TRUE(task.start_time == Time(1672625700, 0));
    EXPECT_TRUE(task.duration == Duration(3600, 0));
    EXPECT_EQ(task.series_id, "7c98b392-2131-4528-92d2-7e7f22d0a9a5");
    EXPECT_EQ(task.process_id, "13aa1c62-64ca-495d-a4b7-84de6a00f56a");
    EXPECT_EQ(task.resource_id, "09d5323d-43e0-4668-b53a-3cf33f9b9a96");
    EXPECT_TRUE(task.deadline == Time(1672632900, 0));
    EXPECT_EQ(task.status, "ongoing");
    EXPECT_TRUE(task.planned_start_time == Time(1672625700, 0));
    EXPECT_TRUE(task.planned_duration == Duration(3600, 0));
    EXPECT_TRUE(task.estimated_duration == Duration(3700, 0));
    EXPECT_TRUE(task.actual_start_time == Time(1672625760, 0));
    EXPECT_TRUE(task.actual_duration == Duration(3760, 0));
    EXPECT_EQ(task.task_details, R"({"some_field": "some_value"})"_json);
  }
}

TEST_F(TestDataTask, equal) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Task task(
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "task_type",
    "This is a task!",
    Time(1672625700, 0),
    Duration(3600, 0),
    "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
    "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
    "09d5323d-43e0-4668-b53a-3cf33f9b9a96",
    Time(1672632900, 0),
    "ongoing",
    Time(1672625700, 0),
    Duration(3600, 0),
    Duration(3700, 0),
    Time(1672625760, 0),
    Duration(3760, 0),
    R"({"some_field": "some_value"})"_json
  );

  {  // Equal
    Task task_to_compare(
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "task_type",
      "This is a task!",
      Time(1672625700, 0),
      Duration(3600, 0),
      "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
      "09d5323d-43e0-4668-b53a-3cf33f9b9a96",
      Time(1672632900, 0),
      "ongoing",
      Time(1672625700, 0),
      Duration(3600, 0),
      Duration(3700, 0),
      Time(1672625760, 0),
      Duration(3760, 0),
      R"({"some_field": "some_value"})"_json
    );

    EXPECT_EQ(task, task_to_compare);
  }

  {  // Different ID
    Task task_to_compare(
      "123456",
      "task_type",
      "This is a task!",
      Time(1672625700, 0),
      Duration(3600, 0),
      "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
      "09d5323d-43e0-4668-b53a-3cf33f9b9a96",
      Time(1672632900, 0),
      "ongoing",
      Time(1672625700, 0),
      Duration(3600, 0),
      Duration(3700, 0),
      Time(1672625760, 0),
      Duration(3760, 0),
      R"({"some_field": "some_value"})"_json
    );

    EXPECT_NE(task, task_to_compare);
  }
}
