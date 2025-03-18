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

#include "rmf2_scheduler/cache/process_handler.hpp"
#include "rmf2_scheduler/cache/task_action.hpp"
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

    // Create isolated task
    task_handler->emplace(
      std::make_shared<data::Task>(
        "isolated_task_1",
        "test_isolated_task",
        data::Time(10 * 60),
        "draft"
      )
    );

    // Create isolated event
    task_handler->emplace(
      std::make_shared<data::Event>(
        "isolated_event_1",
        "test_isolated_event",
        data::Time(10 * 60)
      )
    );

    // Create process and corresponding tasks
    // Add tasks first
    std::vector<data::Task::Ptr> process_tasks(5);
    std::string process_task_prefix = "process_task_";
    std::string process_task_type = "test_process_task";
    std::string process_task_status = "draft";
    std::string process_id = "process_1";
    for (size_t i = 0; i < process_tasks.size(); i++) {
      // Create task_id
      std::ostringstream oss;
      oss << process_task_prefix << i;
      std::string process_task_id = oss.str();

      // Create task
      process_tasks[i] = std::make_shared<data::Task>(
        process_task_id,
        process_task_type,
        data::Time(10 * 60),
        process_task_status
      );

      // Update process ID
      process_tasks[i]->process_id = process_id;

      // Add task to cache
      task_handler->emplace(process_tasks[i]);
    }

    // Create process
    data::Process::Ptr process = std::make_shared<data::Process>();
    process->id = process_id;
    data::Graph & graph = process->graph;
    for (size_t i = 0; i < process_tasks.size(); i++) {
      graph.add_node(process_tasks[i]->id);
    }
    graph.add_edge(process_tasks[0]->id, process_tasks[2]->id, data::Edge("hard"));
    graph.add_edge(process_tasks[1]->id, process_tasks[2]->id, data::Edge("soft"));
    graph.add_edge(process_tasks[2]->id, process_tasks[3]->id, data::Edge("hard"));
    graph.add_edge(process_tasks[2]->id, process_tasks[4]->id, data::Edge("soft"));

    // Add process to cache
    process_handler->emplace(process);

    return cache;
  }
};

}  // namespace cache
}  // namespace rmf2_scheduler

class TestCacheTaskAction : public ::testing::Test
{
};

TEST_F(TestCacheTaskAction, invalid_action) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

  // Create invalid action
  TaskAction process_action(
    action_type::EVENT_ADD,
    ActionPayload()
  );

  // Validate
  std::string error;
  EXPECT_FALSE(process_action.validate(schedule_cache, error));
  EXPECT_EQ(error, "TASK action invalid [EVENT_ADD].");
}

TEST_F(TestCacheTaskAction, undefined_action) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

  // Create event add action
  TaskAction event_action(
    "TASK_INVALID_ACTION",
    ActionPayload()
  );

  // Validate
  std::string error;
  EXPECT_FALSE(event_action.validate(schedule_cache, error));
  EXPECT_EQ(error, "Undefined TASK action [TASK_INVALID_ACTION].");
}

TEST_F(TestCacheTaskAction, task_add) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create task add action
    TaskAction task_action(
      action_type::TASK_ADD,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(task_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "TASK_ADD failed, no task found in payload.");
  }

  {  // Successful
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create task add action
    auto task_to_add = std::make_shared<Task>(
      "task_1",
      "task_type",
      Time(10 * 60),
      "draft"
    );
    TaskAction task_action(
      action_type::TASK_ADD,
      ActionPayload().task(task_to_add)
    );

    // Validate
    std::string error;
    EXPECT_TRUE(task_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());
    EXPECT_FALSE(schedule_cache->has_task("task_1"));

    // Apply
    task_action.apply();
    EXPECT_TRUE(schedule_cache->has_task("task_1"));
    EXPECT_EQ(*schedule_cache->get_task("task_1"), *task_to_add);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"task_1", "add"},
    };
    EXPECT_EQ(
      task_action.record().get("task"),
      expected_record
    );
  }

  {  // task ID already exists
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create task add action
    TaskAction task_action(
      action_type::TASK_ADD,
      ActionPayload().task(
        std::make_shared<Task>(
          "isolated_task_1",
          "test_isolated_task",
          Time(10 * 60),
          "draft"
        )
      )
    );

    // Validate the action again
    std::string error;
    EXPECT_FALSE(task_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "TASK_ADD failed, task [isolated_task_1] already exists.");
  }

  {  // Process ID warning and removal
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create task with process ID
    auto task_to_add = std::make_shared<Task>(
      "task_1",
      "task_type",
      Time(10 * 60),
      "draft"
    );
    task_to_add->process_id = "process_1";

    TaskAction task_action(
      action_type::TASK_ADD,
      ActionPayload().task(task_to_add)
    );

    // Validate
    std::string warning;
    EXPECT_TRUE(task_action.validate(schedule_cache, warning));
    EXPECT_EQ(warning, "TASK_ADD warning, task [task_1]'s process_id is ignored.");

    // Apply
    task_action.apply();
    EXPECT_FALSE(schedule_cache->get_task("task_1")->process_id.has_value());

    // Check added task
    task_to_add->process_id.reset();
    EXPECT_EQ(*schedule_cache->get_task("task_1"), *task_to_add);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"task_1", "add"},
    };
    EXPECT_EQ(
      task_action.record().get("task"),
      expected_record
    );
  }

  {  // Series ID warning and removal
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create task with process ID
    auto task_to_add = std::make_shared<Task>(
      "task_1",
      "task_type",
      Time(10 * 60),
      "draft"
    );
    task_to_add->series_id = "series_1";

    TaskAction task_action(
      action_type::TASK_ADD,
      ActionPayload().task(task_to_add)
    );

    // Validate
    std::string warning;
    EXPECT_TRUE(task_action.validate(schedule_cache, warning));
    EXPECT_EQ(warning, "TASK_ADD warning, task [task_1]'s series_id is ignored.");

    // Apply
    task_action.apply();
    EXPECT_FALSE(schedule_cache->get_task("task_1")->series_id.has_value());

    // Check added task
    task_to_add->series_id.reset();
    EXPECT_EQ(*schedule_cache->get_task("task_1"), *task_to_add);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"task_1", "add"},
    };
    EXPECT_EQ(
      task_action.record().get("task"),
      expected_record
    );
  }
}


TEST_F(TestCacheTaskAction, task_update) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create task update action
    TaskAction task_action(
      action_type::TASK_UPDATE,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(task_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "TASK_UPDATE failed, no task found in payload.");
  }

  {  // Task doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create task update action
    TaskAction task_action(
      action_type::TASK_UPDATE,
      ActionPayload().task(
        std::make_shared<Task>(
          "task_1",
          "task_type",
          Time(10 * 60),
          "draft"
        )
      )
    );

    // Validate
    std::string error;
    EXPECT_FALSE(task_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "TASK_UPDATE failed, task [task_1] doesn't exist.");
    EXPECT_FALSE(schedule_cache->has_task("task_1"));
  }

  {  // Isolated task success
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create update action
    auto task_to_update = std::make_shared<Task>(
      "isolated_task_1",
      "test_isolated_task",
      Time(10 * 60),
      "draft"
    );
    TaskAction task_action(
      action_type::TASK_UPDATE,
      ActionPayload().task(task_to_update)
    );

    // Validate
    std::string error;
    EXPECT_TRUE(task_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    task_action.apply();
    EXPECT_EQ(*schedule_cache->get_task("isolated_task_1"), *task_to_update);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"isolated_task_1", "update"},
    };
    EXPECT_EQ(
      task_action.record().get("task"),
      expected_record
    );
  }

  {  // Isolated task, process ID warning and removal
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create task with process ID
    auto task_to_update = std::make_shared<Task>(
      "isolated_task_1",
      "test_isolated_task",
      Time(10 * 60),
      "draft"
    );
    task_to_update->process_id = "process_1";

    TaskAction task_action(
      action_type::TASK_UPDATE,
      ActionPayload().task(task_to_update)
    );

    // Validate
    std::string warning;
    EXPECT_TRUE(task_action.validate(schedule_cache, warning));
    EXPECT_EQ(
      warning,
      "TASK_UPDATE warning, task [isolated_task_1]'s new process_id is ignored."
    );

    // Apply
    task_action.apply();

    // Check updateded task
    task_to_update->process_id.reset();
    EXPECT_EQ(*schedule_cache->get_task("isolated_task_1"), *task_to_update);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"isolated_task_1", "update"},
    };
    EXPECT_EQ(
      task_action.record().get("task"),
      expected_record
    );
  }

  {  // Isolated task, series ID warning and removal
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create task with series ID
    auto task_to_update = std::make_shared<Task>(
      "isolated_task_1",
      "test_isolated_task",
      Time(10 * 60),
      "draft"
    );
    task_to_update->series_id = "series_1";

    TaskAction task_action(
      action_type::TASK_UPDATE,
      ActionPayload().task(task_to_update)
    );

    // Validate
    std::string warning;
    EXPECT_TRUE(task_action.validate(schedule_cache, warning));
    EXPECT_EQ(
      warning,
      "TASK_UPDATE warning, task [isolated_task_1]'s new series_id is ignored."
    );

    // Apply
    task_action.apply();

    // Check updateded task
    task_to_update->series_id.reset();
    EXPECT_EQ(*schedule_cache->get_task("isolated_task_1"), *task_to_update);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"isolated_task_1", "update"},
    };
    EXPECT_EQ(
      task_action.record().get("task"),
      expected_record
    );
  }

  {  // Process task success
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create task with process ID
    auto task_to_update = std::make_shared<Task>(
      "process_task_1",
      "test_isolated_task",
      Time(10 * 60),
      "draft"
    );
    task_to_update->process_id = "process_1";

    TaskAction task_action(
      action_type::TASK_UPDATE,
      ActionPayload().task(task_to_update)
    );

    // Validate
    std::string error;
    EXPECT_TRUE(task_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());
    std::cout << error << std::endl;

    // Apply
    task_action.apply();
    EXPECT_EQ(*schedule_cache->get_task("process_task_1"), *task_to_update);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"process_task_1", "update"},
    };
    EXPECT_EQ(
      task_action.record().get("task"),
      expected_record
    );
  }

  {  // Process task, process ID warning
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create task with process ID
    auto task_to_update = std::make_shared<Task>(
      "process_task_1",
      "test_isolated_task",
      Time(10 * 60),
      "draft"
    );
    task_to_update->process_id = "process_5";

    TaskAction task_action(
      action_type::TASK_UPDATE,
      ActionPayload().task(task_to_update)
    );

    // Validate
    std::string warning;
    EXPECT_TRUE(task_action.validate(schedule_cache, warning));
    EXPECT_EQ(
      warning,
      "TASK_UPDATE warning, task [process_task_1]'s new process_id is ignored."
    );

    // Apply
    task_action.apply();

    // Check updateded task
    task_to_update->process_id = "process_1";
    EXPECT_EQ(*schedule_cache->get_task("process_task_1"), *task_to_update);

    // Check record
    std::vector<ChangeAction> expected_record {
      {"process_task_1", "update"},
    };
    EXPECT_EQ(
      task_action.record().get("task"),
      expected_record
    );
  }

  // TODO(Briancbn): series tasks
}

TEST_F(TestCacheTaskAction, task_delete) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create task delete action
    TaskAction task_action(
      action_type::TASK_DELETE,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(task_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "TASK_DELETE failed, no ID found in payload.");
  }

  {  // Task doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create task delete action
    TaskAction task_action(
      action_type::TASK_DELETE,
      ActionPayload().id("task_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(task_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "TASK_DELETE failed, task [task_1] doesn't exist.");
  }

  {  // Isolated task success
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create delete action
    TaskAction task_action(
      action_type::TASK_DELETE,
      ActionPayload().id("isolated_task_1")
    );

    // Validate
    std::string error;
    EXPECT_TRUE(task_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    task_action.apply();
    EXPECT_FALSE(schedule_cache->has_task("isolated_task_1"));

    // Check record
    std::vector<ChangeAction> expected_record {
      {"isolated_task_1", "delete"},
    };
    EXPECT_EQ(
      task_action.record().get("task"),
      expected_record
    );
  }

  {  // Process task, process ID warning
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    TaskAction task_action(
      action_type::TASK_DELETE,
      ActionPayload().id("process_task_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(task_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "TASK_DELETE failed, task [process_task_1] is part of a process, "
      "please detach it first."
    );
  }

  // TODO(Briancbn): series tasks
}
