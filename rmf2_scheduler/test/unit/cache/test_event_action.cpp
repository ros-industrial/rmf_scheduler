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

#include "rmf2_scheduler/cache/event_action.hpp"
#include "rmf2_scheduler/cache/process_handler.hpp"
#include "rmf2_scheduler/cache/schedule_cache.hpp"

namespace rmf2_scheduler
{

namespace cache
{

class TestScheduleCache
{
public:
  static rmf2_scheduler::cache::ScheduleCache::UPtr create_empty_cache()
  {
    return std::make_unique<ScheduleCache>();
  }

  static rmf2_scheduler::cache::ScheduleCache::Ptr create_preset_cache1()
  {
    auto cache = std::make_shared<ScheduleCache>();

    // Gain Write access
    ScheduleCache::TaskRestricted task_write_token;
    auto task_handler = cache->make_task_handler(task_write_token);

    ScheduleCache::ProcessRestricted process_write_token;
    auto process_handler = cache->make_process_handler(process_write_token);

    // Create isolated event
    task_handler->emplace(
      std::make_shared<data::Event>(
        "isolated_event_1",
        "test_isolated_event",
        data::Time(10 * 60)
      )
    );

    // Create process and corresponding events
    // Add events first
    std::vector<data::Event::Ptr> process_events(5);
    std::string process_event_prefix = "process_event_";
    std::string process_event_type = "test_process_event";
    std::string process_id = "process_1";
    for (size_t i = 0; i < process_events.size(); i++) {
      // Create event_id
      std::ostringstream oss;
      oss << process_event_prefix << i;
      std::string process_event_id = oss.str();

      // Create event
      process_events[i] = std::make_shared<data::Event>(
        process_event_id,
        process_event_type,
        data::Time(10 * 60)
      );

      // Update process ID
      process_events[i]->process_id = process_id;

      // Add event to cache
      task_handler->emplace(process_events[i]);
    }

    // Create process
    data::Process::Ptr process = std::make_shared<data::Process>();
    process->id = process_id;
    data::Graph & graph = process->graph;
    for (size_t i = 0; i < process_events.size(); i++) {
      graph.add_node(process_events[i]->id);
    }
    graph.add_edge(process_events[0]->id, process_events[2]->id, data::Edge("hard"));
    graph.add_edge(process_events[1]->id, process_events[2]->id, data::Edge("soft"));
    graph.add_edge(process_events[2]->id, process_events[3]->id, data::Edge("hard"));
    graph.add_edge(process_events[2]->id, process_events[4]->id, data::Edge("soft"));

    // Add process to cache
    process_handler->emplace(process);

    return cache;
  }
};

}  // namespace cache
}  // namespace rmf2_scheduler

class TestCacheEventAction : public ::testing::Test
{
};

TEST_F(TestCacheEventAction, invalid_action) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

  // Create invalid action
  EventAction event_action(
    action_type::PROCESS_ADD,
    ActionPayload()
  );

  // Validate
  std::string error;
  EXPECT_FALSE(event_action.validate(schedule_cache, error));
  EXPECT_EQ(error, "EVENT action invalid [PROCESS_ADD].");
}

TEST_F(TestCacheEventAction, undefined_action) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

  // Create event add action
  EventAction event_action(
    "EVENT_INVALID_OPERATION",
    ActionPayload()
  );

  // Validate
  std::string error;
  EXPECT_FALSE(event_action.validate(schedule_cache, error));
  EXPECT_EQ(error, "Undefined EVENT action [EVENT_INVALID_OPERATION].");
}

TEST_F(TestCacheEventAction, event_add) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create event add action
    EventAction event_action(
      action_type::EVENT_ADD,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(event_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "EVENT_ADD failed, no event found in payload.");
  }

  {  // Successful
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create event add action
    auto event_to_add = std::make_shared<Event>(
      "task_1",
      "task_type",
      Time(10 * 60)
    );
    EventAction event_action(
      action_type::EVENT_ADD,
      ActionPayload().event(event_to_add)
    );

    // Validate
    std::string error;
    EXPECT_TRUE(event_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());
    EXPECT_FALSE(schedule_cache->has_event("task_1"));

    // Apply
    event_action.apply();
    EXPECT_TRUE(schedule_cache->has_event("task_1"));
    EXPECT_EQ(*schedule_cache->get_event("task_1"), *event_to_add);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"task_1", "add"},
    };
    EXPECT_EQ(
      event_action.record().get("event"),
      expected_record
    );
  }

  {  // event ID already exists
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create 1st event add action
    EventAction event_action(
      action_type::EVENT_ADD,
      ActionPayload().event(
        std::make_shared<Event>(
          "isolated_event_1",
          "task_type",
          Time(10 * 60)
        )
      )
    );

    // Validate the action again
    std::string error;
    EXPECT_FALSE(event_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "EVENT_ADD failed, event [isolated_event_1] already exists.");
  }

  {  // Process ID warning and removal
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create event with process ID
    auto event_to_add = std::make_shared<Event>(
      "task_1",
      "task_type",
      Time(10 * 60)
    );
    event_to_add->process_id = "process_1";

    EventAction event_action(
      action_type::EVENT_ADD,
      ActionPayload().event(event_to_add)
    );

    // Validate
    std::string warning;
    EXPECT_TRUE(event_action.validate(schedule_cache, warning));
    EXPECT_EQ(warning, "EVENT_ADD warning, event [task_1]'s process_id is ignored.");

    // Apply
    event_action.apply();
    EXPECT_FALSE(schedule_cache->get_event("task_1")->process_id.has_value());

    // Check added event
    event_to_add->process_id.reset();
    EXPECT_EQ(*schedule_cache->get_event("task_1"), *event_to_add);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"task_1", "add"},
    };
    EXPECT_EQ(
      event_action.record().get("event"),
      expected_record
    );
  }

  {  // Series ID warning and removal
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create event with process ID
    auto event_to_add = std::make_shared<Event>(
      "task_1",
      "task_type",
      Time(10 * 60)
    );
    event_to_add->series_id = "series_1";

    EventAction event_action(
      action_type::EVENT_ADD,
      ActionPayload().event(event_to_add)
    );

    // Validate
    std::string warning;
    EXPECT_TRUE(event_action.validate(schedule_cache, warning));
    EXPECT_EQ(warning, "EVENT_ADD warning, event [task_1]'s series_id is ignored.");

    // Apply
    event_action.apply();
    EXPECT_FALSE(schedule_cache->get_event("task_1")->series_id.has_value());

    // Check added event
    event_to_add->series_id.reset();
    EXPECT_EQ(*schedule_cache->get_event("task_1"), *event_to_add);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"task_1", "add"},
    };
    EXPECT_EQ(
      event_action.record().get("event"),
      expected_record
    );
  }
}


TEST_F(TestCacheEventAction, event_update) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create event update action
    EventAction event_action(
      action_type::EVENT_UPDATE,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(event_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "EVENT_UPDATE failed, no event found in payload.");
  }

  {  // Event doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create event update action
    EventAction event_action(
      action_type::EVENT_UPDATE,
      ActionPayload().event(
        std::make_shared<Event>(
          "task_1",
          "task_type",
          Time(10 * 60)
        )
      )
    );

    // Validate
    std::string error;
    EXPECT_FALSE(event_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "EVENT_UPDATE failed, event [task_1] doesn't exist.");
    EXPECT_FALSE(schedule_cache->has_event("task_1"));
  }

  {  // Isolated event success
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create update action
    auto event_to_update = std::make_shared<Event>(
      "isolated_event_1",
      "task_type",
      Time(10 * 60)
    );
    EventAction event_action(
      action_type::EVENT_UPDATE,
      ActionPayload().event(event_to_update)
    );

    // Validate
    std::string error;
    EXPECT_TRUE(event_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    event_action.apply();
    EXPECT_EQ(*schedule_cache->get_event("isolated_event_1"), *event_to_update);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"isolated_event_1", "update"},
    };
    EXPECT_EQ(
      event_action.record().get("event"),
      expected_record
    );
  }

  {  // Isolated event, process ID warning and removal
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create event with process ID
    auto event_to_update = std::make_shared<Event>(
      "isolated_event_1",
      "task_type",
      Time(10 * 60)
    );
    event_to_update->process_id = "process_1";

    EventAction event_action(
      action_type::EVENT_UPDATE,
      ActionPayload().event(event_to_update)
    );

    // Validate
    std::string warning;
    EXPECT_TRUE(event_action.validate(schedule_cache, warning));
    EXPECT_EQ(
      warning,
      "EVENT_UPDATE warning, event [isolated_event_1]'s new process_id is ignored."
    );

    // Apply
    event_action.apply();

    // Check updateded event
    event_to_update->process_id.reset();
    EXPECT_EQ(*schedule_cache->get_event("isolated_event_1"), *event_to_update);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"isolated_event_1", "update"},
    };
    EXPECT_EQ(
      event_action.record().get("event"),
      expected_record
    );
  }

  {  // Isolated event, series ID warning and removal
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create event with series ID
    auto event_to_update = std::make_shared<Event>(
      "isolated_event_1",
      "task_type",
      Time(10 * 60)
    );
    event_to_update->series_id = "series_1";

    EventAction event_action(
      action_type::EVENT_UPDATE,
      ActionPayload().event(event_to_update)
    );

    // Validate
    std::string warning;
    EXPECT_TRUE(event_action.validate(schedule_cache, warning));
    EXPECT_EQ(
      warning,
      "EVENT_UPDATE warning, event [isolated_event_1]'s new series_id is ignored."
    );

    // Apply
    event_action.apply();

    // Check updateded event
    event_to_update->series_id.reset();
    EXPECT_EQ(*schedule_cache->get_event("isolated_event_1"), *event_to_update);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"isolated_event_1", "update"},
    };
    EXPECT_EQ(
      event_action.record().get("event"),
      expected_record
    );
  }

  {  // Process event success
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create event with process ID
    auto event_to_update = std::make_shared<Event>(
      "process_event_1",
      "task_type",
      Time(10 * 60)
    );
    event_to_update->process_id = "process_1";

    EventAction event_action(
      action_type::EVENT_UPDATE,
      ActionPayload().event(event_to_update)
    );

    // Validate
    std::string error;
    EXPECT_TRUE(event_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    event_action.apply();
    EXPECT_EQ(*schedule_cache->get_event("process_event_1"), *event_to_update);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"process_event_1", "update"},
    };
    EXPECT_EQ(
      event_action.record().get("event"),
      expected_record
    );
  }

  {  // Process event, process ID warning
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create event with process ID
    auto event_to_update = std::make_shared<Event>(
      "process_event_1",
      "task_type",
      Time(10 * 60)
    );
    event_to_update->process_id = "process_5";

    EventAction event_action(
      action_type::EVENT_UPDATE,
      ActionPayload().event(event_to_update)
    );

    // Validate
    std::string warning;
    EXPECT_TRUE(event_action.validate(schedule_cache, warning));
    EXPECT_EQ(
      warning,
      "EVENT_UPDATE warning, event [process_event_1]'s new process_id is ignored."
    );

    // Apply
    event_action.apply();

    // Check updateded event
    event_to_update->process_id = "process_1";
    EXPECT_EQ(*schedule_cache->get_event("process_event_1"), *event_to_update);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"process_event_1", "update"},
    };
    EXPECT_EQ(
      event_action.record().get("event"),
      expected_record
    );
  }

  // TODO(Briancbn): series events
}

TEST_F(TestCacheEventAction, event_delete) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create event delete action
    EventAction event_action(
      action_type::EVENT_DELETE,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(event_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "EVENT_DELETE failed, no ID found in payload.");
  }

  {  // Event doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create event delete action
    EventAction event_action(
      action_type::EVENT_DELETE,
      ActionPayload().id("task_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(event_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "EVENT_DELETE failed, event [task_1] doesn't exist.");
  }

  {  // Isolated event success
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create delete action
    EventAction event_action(
      action_type::EVENT_DELETE,
      ActionPayload().id("isolated_event_1")
    );

    // Validate
    std::string error;
    EXPECT_TRUE(event_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    event_action.apply();
    EXPECT_FALSE(schedule_cache->has_event("isolated_event_1"));

    // Check record
    std::vector<ChangeAction> expected_record {
      {"isolated_event_1", "delete"},
    };
    EXPECT_EQ(
      event_action.record().get("event"),
      expected_record
    );
  }

  {  // Process event, process ID warning
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    EventAction event_action(
      action_type::EVENT_DELETE,
      ActionPayload().id("process_event_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(event_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "EVENT_DELETE failed, event [process_event_1] is part of a process, "
      "please detach it first."
    );
  }

  // TODO(Briancbn): series events
}
