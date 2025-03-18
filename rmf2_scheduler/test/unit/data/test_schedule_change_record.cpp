// Copyright 2025 ROS Industrial Consortium Asia Pacific
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
#include "../../utils/gtest_macros.hpp"
#include "rmf2_scheduler/data/schedule_change_record.hpp"

class TestScheduleChangeRecord : public ::testing::Test
{
};

TEST_F(TestScheduleChangeRecord, change_action_equal) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  ChangeAction change_action{
    "event_1",
    "add"
  };

  {  // Equal
    ChangeAction change_action_to_compare{
      "event_1",
      "add"
    };

    EXPECT_EQ(change_action, change_action_to_compare);
  }

  {  // Different action
    ChangeAction change_action_to_compare{
      "event_1",
      "update"
    };

    EXPECT_NE(change_action, change_action_to_compare);
  }
}

TEST_F(TestScheduleChangeRecord, crud) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  ScheduleChangeRecord record;

  // Add record
  record.add(
    "event", {
    {"event_1", "add"},
    {"event_3", "update"},
    {"event_4", "delete"},
  });
  record.add(
    "task", {
    {"process_task_1", "add"},
    {"series_task_2", "update"}
  });
  record.add(
    "process", {
    {"process_1", "delete"}
  });
  record.add(
    "series", {
    {"series_1", "add"},
    {"series_5", "delete"}
  });

  // Check event record
  std::vector<ChangeAction>
  expected_event_record {
    {"event_1", "add"},
    {"event_3", "update"},
    {"event_4", "delete"},
    {"process_task_1", "add"},
    {"series_task_2", "update"}
  };
  EXPECT_EQ(
    record.get("event"),
    expected_event_record
  );

  // Check process record
  std::vector<ChangeAction>
  expected_process_record {
    {"process_1", "delete"}
  };
  EXPECT_EQ(
    record.get("process"),
    expected_process_record
  );

  // Check series record
  std::vector<ChangeAction>
  expected_series_record {
    {"series_1", "add"},
    {"series_5", "delete"}
  };
  EXPECT_EQ(
    record.get("series"),
    expected_series_record
  );
}

TEST_F(TestScheduleChangeRecord, crud_exceptions) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  ScheduleChangeRecord record;

  EXPECT_THROW_EQ(
    record.add("invalid_type", {}),
    std::invalid_argument(
      "ScheduleChangeRecord add failed, type [invalid_type] is invalid."
    )
  );

  EXPECT_THROW_EQ(
    record.get("invalid_type"),
    std::invalid_argument(
      "ScheduleChangeRecord get failed, type [invalid_type] is invalid."
    )
  );
}

TEST_F(TestScheduleChangeRecord, squash) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  std::vector<ScheduleChangeRecord> records(3);

  // Add record
  records[0].add(
    "event", {
    {"event_1", "add"},
    {"event_3", "update"},
    {"event_4", "delete"},
    {"process_task_1", "add"},
    {"series_task_2", "update"},
  });
  records[0].add(
    "process", {
    {"process_1", "delete"},
  });
  records[0].add(
    "series", {
    {"series_1", "add"},
    {"series_5", "delete"},
  });

  records[1].add(
    "event", {
    {"event_1", "update"},
    {"event_2", "update"},
    {"event_3", "delete"},
    {"event_4", "add"},
    {"event_5", "delete"},
    {"event_6", "add"},
    {"series_task_2", "update"},
  }
  );

  records[1].add(
    "series", {
    {"series_5", "add"},
    {"series_5", "update"},
  });

  records[2].add(
    "event", {
    {"event_1", "delete"},
    {"event_4", "update"},
    {"event_5", "add"},
    {"event_6", "update"},
  });

  records[2].add(
    "process", {
    {"process_2", "add"},
    {"process_2", "update"},
  });


  // Squash
  auto squashed_record = ScheduleChangeRecord::squash(records);

  // Check event record
  std::vector<ChangeAction>
  expected_event_record {
    {"event_2", "update"},
    {"event_3", "delete"},
    {"event_4", "update"},
    {"event_5", "update"},
    {"event_6", "add"},
    {"process_task_1", "add"},
    {"series_task_2", "update"}
  };
  EXPECT_EQ(
    squashed_record.get("event"),
    expected_event_record
  );

  // Check process record
  std::vector<ChangeAction>
  expected_process_record {
    {"process_1", "delete"},
    {"process_2", "add"},
  };
  EXPECT_EQ(
    squashed_record.get("process"),
    expected_process_record
  );

  // Check series record
  std::vector<ChangeAction>
  expected_series_record {
    {"series_1", "add"},
    {"series_5", "update"}
  };
  EXPECT_EQ(
    squashed_record.get("series"),
    expected_series_record
  );
}
