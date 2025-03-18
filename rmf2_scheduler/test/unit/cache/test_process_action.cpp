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
#include "rmf2_scheduler/cache/process_action.hpp"
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
      oss << process_event_prefix << i + 1;
      std::string process_event_id = oss.str();

      // Create event
      process_events[i] = std::make_shared<data::Event>(
        process_event_id,
        process_event_type,
        data::Time(10 * 60, 0)
      );
      process_events[i]->duration = std::chrono::minutes(5);

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


class TestCacheProcessAction : public ::testing::Test
{
};

TEST_F(TestCacheProcessAction, invalid_action) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

  // Create invalid action
  ProcessAction process_action(
    action_type::EVENT_ADD,
    ActionPayload()
  );

  // Validate
  std::string error;
  EXPECT_FALSE(process_action.validate(schedule_cache, error));
  EXPECT_EQ(error, "PROCESS action invalid [EVENT_ADD].");
}

TEST_F(TestCacheProcessAction, undefined_action) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

  // Create invalid process action
  ProcessAction event_action(
    "PROCESS_INVALID_ACTION",
    ActionPayload()
  );

  // Validate
  std::string error;
  EXPECT_FALSE(event_action.validate(schedule_cache, error));
  EXPECT_EQ(error, "Undefined PROCESS action [PROCESS_INVALID_ACTION].");
}

TEST_F(TestCacheProcessAction, process_add) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create process add action
    ProcessAction process_action(
      action_type::PROCESS_ADD,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ADD failed, no process found in payload.");
  }

  {  // Invalid process ID
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();
    auto process_to_add = std::make_shared<Process>();
    process_to_add->id = "process_1";
    ProcessAction process_action(
      action_type::PROCESS_ADD,
      ActionPayload().process(process_to_add)
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ADD failed, process [process_1] already exists.");
  }

  {  // node doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();
    auto process_to_add = std::make_shared<Process>();
    process_to_add->id = "process_1";
    process_to_add->graph.add_node("event_1");
    ProcessAction process_action(
      action_type::PROCESS_ADD,
      ActionPayload().process(process_to_add)
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ADD failed, node [event_1] doesn't exist as task.");
  }

  {  // node is part of another process
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();
    auto process_to_add = std::make_shared<Process>();
    process_to_add->id = "process_2";
    process_to_add->graph.add_node("process_event_1");
    ProcessAction process_action(
      action_type::PROCESS_ADD,
      ActionPayload().process(process_to_add)
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "PROCESS_ADD failed, task [process_event_1] is already part of a process, "
      "please detach it first."
    );
  }

  {  // Successful
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Add 3 events using EventAction
    std::vector<Event::Ptr> events_to_add(3);
    std::string event_id_prefix = "process_event_";
    std::string event_type = "test_event";
    for (int i = 0; i < 3; i++) {
      // Create event_id
      std::ostringstream oss;
      oss << event_id_prefix << i;
      std::string event_id = oss.str();
      auto event_to_add = std::make_shared<Event>(
        event_id,
        event_type,
        Time(10 * 60)
      );

      EventAction event_action(
        action_type::EVENT_ADD,
        ActionPayload().event(event_to_add)
      );

      // Validate, should always be true
      std::string error;
      ASSERT_TRUE(event_action.validate(schedule_cache, error));
      ASSERT_TRUE(error.empty());

      // Apply
      event_action.apply();

      // Keep the events for checking purpose
      events_to_add[i] = event_to_add;
    }

    // Define a process involving these 3 events
    Process::Ptr process_to_add = std::make_shared<Process>();
    process_to_add->id = "process_1";
    for (int i = 0; i < 3; i++) {
      process_to_add->graph.add_node(events_to_add[i]->id);
    }
    process_to_add->graph.add_edge(events_to_add[0]->id, events_to_add[1]->id);
    process_to_add->graph.add_edge(events_to_add[0]->id, events_to_add[2]->id);

    // Create the process action
    ProcessAction process_action(
      action_type::PROCESS_ADD,
      ActionPayload().process(process_to_add)
    );

    // Validate
    std::string error;
    EXPECT_TRUE(process_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    process_action.apply();

    // Check the process added
    EXPECT_TRUE(schedule_cache->has_process(process_to_add->id));
    EXPECT_NE(schedule_cache->get_process(process_to_add->id), process_to_add);
    EXPECT_EQ(*schedule_cache->get_process(process_to_add->id), *process_to_add);

    // Check the events' process id has changed
    EXPECT_EQ(schedule_cache->get_event(events_to_add[0]->id)->process_id, process_to_add->id);
    EXPECT_EQ(schedule_cache->get_event(events_to_add[1]->id)->process_id, process_to_add->id);
    EXPECT_EQ(schedule_cache->get_event(events_to_add[2]->id)->process_id, process_to_add->id);

    // Check record
    std::vector<ChangeAction> expected_event_record {
      {events_to_add[0]->id, "update"},
      {events_to_add[1]->id, "update"},
      {events_to_add[2]->id, "update"},
    };
    EXPECT_EQ(
      process_action.record().get("event"),
      expected_event_record
    );

    std::vector<ChangeAction> expected_process_record {
      {process_to_add->id, "add"},
    };
    EXPECT_EQ(
      process_action.record().get("process"),
      expected_process_record
    );
  }
}

TEST_F(TestCacheProcessAction, process_attach_node) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload no ID
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create empty process attach node action
    ProcessAction process_action(
      action_type::PROCESS_ATTACH_NODE,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ATTACH_NODE failed, no process ID found in payload.");
  }

  {  // Invalid payload no node
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    ProcessAction process_action(
      action_type::PROCESS_ATTACH_NODE,
      ActionPayload().id("process_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ATTACH_NODE failed, no node found in payload.");
  }

  {  // Process doesn't exist
    // Create preset cache
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create process attach node action
    ProcessAction process_action(
      action_type::PROCESS_ATTACH_NODE,
      ActionPayload().id("process_2").node_id("event_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ATTACH_NODE failed, process [process_2] doesn't exist.");
  }

  {  // Event already part of the process
    // Create preset cache
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create process attach node action
    ProcessAction process_action(
      action_type::PROCESS_ATTACH_NODE,
      ActionPayload().id("process_1").node_id("process_event_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "PROCESS_ATTACH_NODE failed, node [process_event_1] "
      "is already part of the process."
    );
  }

  {  // Event doesn't exist
    // Create preset cache
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create process attach node action
    ProcessAction process_action(
      action_type::PROCESS_ATTACH_NODE,
      ActionPayload().id("process_1").node_id("event_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ATTACH_NODE failed, node [event_1] doesn't exist as task.");
  }

  {  // Event belong to another process
    // Create preset cache
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create an empty process first
    auto process = std::make_shared<Process>();
    process->id = "process_2";
    ProcessAction process_add_action(
      action_type::PROCESS_ADD,
      ActionPayload().process(process)
    );

    // Validate
    std::string error;
    ASSERT_TRUE(process_add_action.validate(schedule_cache, error));
    ASSERT_TRUE(error.empty());

    // Apply
    process_add_action.apply();

    // Create process attach node action
    ProcessAction process_action(
      action_type::PROCESS_ATTACH_NODE,
      ActionPayload().id("process_2").node_id("process_event_1")
    );

    // Validate
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "PROCESS_ATTACH_NODE failed, task [process_event_1] "
      "is already part of another process, please detach it first."
    );
  }

  {  // Success
    // Create preset cache
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create process attach node action
    ProcessAction process_action(
      action_type::PROCESS_ATTACH_NODE,
      ActionPayload().id("process_1").node_id("isolated_event_1")
    );

    // Validate
    std::string error;
    EXPECT_TRUE(process_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    process_action.apply();

    // Check process is updated
    auto process = schedule_cache->get_process("process_1");
    EXPECT_TRUE(process->graph.has_node("isolated_event_1"));

    // Check event is updated
    auto event = schedule_cache->get_event("isolated_event_1");
    EXPECT_EQ(*event->process_id, "process_1");

    // Check record
    std::vector<ChangeAction> expected_event_record {
      {"isolated_event_1", "update"},
    };
    EXPECT_EQ(
      process_action.record().get("event"),
      expected_event_record
    );

    std::vector<ChangeAction> expected_process_record {
      {"process_1", "update"},
    };
    EXPECT_EQ(
      process_action.record().get("process"),
      expected_process_record
    );
  }
}

TEST_F(TestCacheProcessAction, process_add_dependency) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload no source
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create empty process action
    ProcessAction process_action(
      action_type::PROCESS_ADD_DEPENDENCY,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ADD_DEPENDENCY failed, no source found in payload.");
  }

  {  // Invalid payload no destination
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    ProcessAction process_action(
      action_type::PROCESS_ADD_DEPENDENCY,
      ActionPayload().source_id("process_event_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ADD_DEPENDENCY failed, no destination found in payload.");
  }

  {  // Invalid payload no edge type
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    ProcessAction process_action(
      action_type::PROCESS_ADD_DEPENDENCY,
      ActionPayload()
      .source_id("process_event_1")
      .destination_id("process_event_2")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ADD_DEPENDENCY failed, no edge property found in payload.");
  }

  {  // source event doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    ProcessAction process_action(
      action_type::PROCESS_ADD_DEPENDENCY,
      ActionPayload()
      .source_id("event_1")
      .destination_id("process_event_2")
      .edge_type("hard")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ADD_DEPENDENCY failed, node [event_1] doesn't exist as task.");
  }

  {  // destination event doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    ProcessAction process_action(
      action_type::PROCESS_ADD_DEPENDENCY,
      ActionPayload()
      .source_id("process_event_1")
      .destination_id("event_2")
      .edge_type("hard")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_ADD_DEPENDENCY failed, node [event_2] doesn't exist as task.");
  }

  {  // events are not part of the same process
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    ProcessAction process_action(
      action_type::PROCESS_ADD_DEPENDENCY,
      ActionPayload()
      .source_id("isolated_event_1")
      .destination_id("process_event_2")
      .edge_type("hard")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "PROCESS_ADD_DEPENDENCY failed, source and destination node "
      "doesn't belong to the same process."
    );
  }

  {  // Edge already exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    ProcessAction process_action(
      action_type::PROCESS_ADD_DEPENDENCY,
      ActionPayload()
      .source_id("process_event_1")
      .destination_id("process_event_3")
      .edge_type("hard")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "PROCESS_ADD_DEPENDENCY failed, edge [process_event_1] to [process_event_3] already exists."
    );
  }

  {  // Success
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    ProcessAction process_action(
      action_type::PROCESS_ADD_DEPENDENCY,
      ActionPayload()
      .source_id("process_event_1")
      .destination_id("process_event_2")
      .edge_type("hard")
    );

    // Validate
    std::string error;
    EXPECT_TRUE(process_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    process_action.apply();

    // Check if the edge is added
    auto process = schedule_cache->get_process("process_1");
    auto successors = process->graph.get_node("process_event_1")->outbound_edges();
    EXPECT_TRUE(successors.find("process_event_2") != successors.end());

    // Check record
    EXPECT_TRUE(process_action.record().get("event").empty());

    std::vector<ChangeAction> expected_process_record {
      {"process_1", "update"},
    };
    EXPECT_EQ(
      process_action.record().get("process"),
      expected_process_record
    );
  }
}

TEST_F(TestCacheProcessAction, process_update) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create process add action
    ProcessAction process_action(
      action_type::PROCESS_UPDATE,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_UPDATE failed, no process found in payload.");
  }

  {  // Invalid process ID
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();
    auto process = std::make_shared<Process>();
    process->id = "process_1";
    ProcessAction process_action(
      action_type::PROCESS_UPDATE,
      ActionPayload().process(process)
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_UPDATE failed, process [process_1] doesn't exist.");
  }

  {  // node doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();
    auto process = std::make_shared<Process>();
    process->id = "process_1";
    process->graph.add_node("event_1");
    ProcessAction process_action(
      action_type::PROCESS_UPDATE,
      ActionPayload().process(process)
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_UPDATE failed, node [event_1] doesn't exist as task.");
  }

  {  // node is part of another process
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create process 2
    auto process_to_add = std::make_shared<Process>();
    process_to_add->id = "process_2";
    process_to_add->graph.add_node("isolated_event_1");
    ProcessAction process_add_action(
      action_type::PROCESS_ADD,
      ActionPayload().process(process_to_add)
    );
    std::string error;
    ASSERT_TRUE(process_add_action.validate(schedule_cache, error));
    ASSERT_TRUE(error.empty());
    process_add_action.apply();

    // Create process action
    auto process_to_update = std::make_shared<Process>(
      *schedule_cache->get_process("process_1")
    );
    process_to_update->graph.add_node("isolated_event_1");
    process_to_update->graph.add_edge("isolated_event_1", "process_event_1");
    process_to_update->graph.delete_node("process_event_4");

    ProcessAction process_update_action(
      action_type::PROCESS_UPDATE,
      ActionPayload().process(process_to_update)
    );

    // Validate
    EXPECT_FALSE(process_update_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "PROCESS_UPDATE failed, task [isolated_event_1] is part of another process, "
      "please detach it first."
    );
  }

  {  // Successful
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();


    // Create process action
    auto process_to_update = std::make_shared<Process>(
      *schedule_cache->get_process("process_1")
    );
    process_to_update->graph.add_node("isolated_event_1");
    process_to_update->graph.add_edge("isolated_event_1", "process_event_1");
    process_to_update->graph.delete_node("process_event_4");

    // Create the process action
    ProcessAction process_action(
      action_type::PROCESS_UPDATE,
      ActionPayload().process(process_to_update)
    );

    // Validate
    std::string error;
    EXPECT_TRUE(process_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    process_action.apply();

    // Check the process added
    EXPECT_NE(schedule_cache->get_process(process_to_update->id), process_to_update);
    EXPECT_EQ(*schedule_cache->get_process(process_to_update->id), *process_to_update);

    // Check the events' process id has changed
    EXPECT_EQ(schedule_cache->get_event("isolated_event_1")->process_id, "process_1");
    EXPECT_FALSE(schedule_cache->get_event("process_event_4")->process_id.has_value());

    // Check record
    std::vector<ChangeAction> expected_event_record {
      {"isolated_event_1", "update"},
      {"process_event_4", "update"},
    };
    EXPECT_EQ(
      process_action.record().get("event"),
      expected_event_record
    );

    std::vector<ChangeAction> expected_process_record {
      {"process_1", "update"},
    };

    EXPECT_EQ(
      process_action.record().get("process"),
      expected_process_record
    );
  }
}

TEST_F(TestCacheProcessAction, process_update_start_time) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create process update start time action
    ProcessAction process_action(
      action_type::PROCESS_UPDATE_START_TIME,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_UPDATE_START_TIME failed, no process ID found in payload.");
  }

  {  // Invalid process ID
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();
    ProcessAction process_action(
      action_type::PROCESS_UPDATE_START_TIME,
      ActionPayload().id("process_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_UPDATE_START_TIME failed, process [process_1] doesn't exist.");
  }

  {  // process is cyclic
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Make process cyclic
    ProcessAction process_add_dependency_action(
      action_type::PROCESS_ADD_DEPENDENCY,
      ActionPayload()
      .source_id("process_event_4")
      .destination_id("process_event_1")
      .edge_type("hard")
    );
    std::string error;
    ASSERT_TRUE(process_add_dependency_action.validate(schedule_cache, error));
    ASSERT_TRUE(error.empty());
    process_add_dependency_action.apply();

    // Create process action
    ProcessAction process_action(
      action_type::PROCESS_UPDATE_START_TIME,
      ActionPayload().id("process_1")
    );

    // Validate
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "PROCESS_UPDATE_START_TIME failed, process [process_1] is cyclic."
    );
  }

  {  // Successful
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create process action
    ProcessAction process_action(
      action_type::PROCESS_UPDATE_START_TIME,
      ActionPayload().id("process_1")
    );
    // Validate
    std::string error;
    EXPECT_TRUE(process_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    process_action.apply();

    // Check the events start time
    EXPECT_EQ(schedule_cache->get_event("process_event_1")->start_time, Time(10 * 60, 0));
    EXPECT_EQ(schedule_cache->get_event("process_event_2")->start_time, Time(10 * 60, 0));
    EXPECT_EQ(schedule_cache->get_event("process_event_3")->start_time, Time(15 * 60, 0));
    EXPECT_EQ(schedule_cache->get_event("process_event_4")->start_time, Time(20 * 60, 0));
    EXPECT_EQ(schedule_cache->get_event("process_event_5")->start_time, Time(20 * 60, 0));

    // Check record
    std::vector<ChangeAction> expected_event_record {
      {"process_event_1", "update"},
      {"process_event_2", "update"},
      {"process_event_3", "update"},
      {"process_event_4", "update"},
      {"process_event_5", "update"},
    };
    EXPECT_EQ(
      process_action.record().get("event"),
      expected_event_record
    );

    EXPECT_TRUE(process_action.record().get("process").empty());
  }
}

TEST_F(TestCacheProcessAction, process_detach_node) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload no node
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    ProcessAction process_action(
      action_type::PROCESS_DETACH_NODE,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_DETACH_NODE failed, no node found in payload.");
  }

  {  // Event doesn't exist
    // Create preset cache
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create process detach node action
    ProcessAction process_action(
      action_type::PROCESS_DETACH_NODE,
      ActionPayload().node_id("event_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_DETACH_NODE failed, node [event_1] doesn't exist as task.");
  }

  {  // Event not part of a process
    // Create preset cache
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create process detach node action
    ProcessAction process_action(
      action_type::PROCESS_DETACH_NODE,
      ActionPayload().node_id("isolated_event_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "PROCESS_DETACH_NODE failed, node [isolated_event_1] doesn't belong to a process."
    );
  }

  {  // Success
    // Create preset cache
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create process detach node action
    ProcessAction process_action(
      action_type::PROCESS_DETACH_NODE,
      ActionPayload().node_id("process_event_1")
    );

    // Validate
    std::string error;
    EXPECT_TRUE(process_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    process_action.apply();

    // Check process is updated
    auto process = schedule_cache->get_process("process_1");
    EXPECT_FALSE(process->graph.has_node("process_event_1"));

    // Check event is updated
    auto event = schedule_cache->get_event("process_event_1");
    EXPECT_FALSE(event->process_id.has_value());

    // Check record
    std::vector<ChangeAction> expected_event_record {
      {"process_event_1", "update"},
    };
    EXPECT_EQ(
      process_action.record().get("event"),
      expected_event_record
    );

    std::vector<ChangeAction> expected_process_record {
      {"process_1", "update"},
    };
    EXPECT_EQ(
      process_action.record().get("process"),
      expected_process_record
    );
  }
}

TEST_F(TestCacheProcessAction, process_delete_dependency) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload no source
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create empty process action
    ProcessAction process_action(
      action_type::PROCESS_DELETE_DEPENDENCY,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_DELETE_DEPENDENCY failed, no source found in payload.");
  }

  {  // Invalid payload no destination
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    ProcessAction process_action(
      action_type::PROCESS_DELETE_DEPENDENCY,
      ActionPayload().source_id("process_event_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_DELETE_DEPENDENCY failed, no destination found in payload.");
  }

  {  // source event doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    ProcessAction process_action(
      action_type::PROCESS_DELETE_DEPENDENCY,
      ActionPayload()
      .source_id("event_1")
      .destination_id("process_event_2")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_DELETE_DEPENDENCY failed, node [event_1] doesn't exist as task.");
  }

  {  // destination event doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    ProcessAction process_action(
      action_type::PROCESS_DELETE_DEPENDENCY,
      ActionPayload()
      .source_id("process_event_1")
      .destination_id("event_2")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_DELETE_DEPENDENCY failed, node [event_2] doesn't exist as task.");
  }

  {  // events are not part of the same process
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    ProcessAction process_action(
      action_type::PROCESS_DELETE_DEPENDENCY,
      ActionPayload()
      .source_id("isolated_event_1")
      .destination_id("process_event_2")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "PROCESS_DELETE_DEPENDENCY failed, source and destination node "
      "doesn't belong to the same process."
    );
  }

  {  // Edge doesn't exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    ProcessAction process_action(
      action_type::PROCESS_DELETE_DEPENDENCY,
      ActionPayload()
      .source_id("process_event_1")
      .destination_id("process_event_4")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "PROCESS_DELETE_DEPENDENCY failed, edge [process_event_1] to [process_event_4] doesn't exist."
    );
  }

  {  // Success
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    ProcessAction process_action(
      action_type::PROCESS_DELETE_DEPENDENCY,
      ActionPayload()
      .source_id("process_event_1")
      .destination_id("process_event_3")
    );

    // Validate
    std::string error;
    EXPECT_TRUE(process_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    process_action.apply();

    // Check if the edge is deleted
    auto process = schedule_cache->get_process("process_1");
    auto successors = process->graph.get_node("process_event_1")->outbound_edges();
    EXPECT_TRUE(successors.find("process_event_3") == successors.end());

    // Check record
    EXPECT_TRUE(process_action.record().get("event").empty());

    std::vector<ChangeAction> expected_process_record {
      {"process_1", "update"},
    };
    EXPECT_EQ(
      process_action.record().get("process"),
      expected_process_record
    );
  }
}

TEST_F(TestCacheProcessAction, process_delete) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create process delete action
    ProcessAction process_action(
      action_type::PROCESS_DELETE,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_DELETE failed, no ID found in payload.");
  }

  {  // Invalid process ID
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();
    ProcessAction process_action(
      action_type::PROCESS_DELETE,
      ActionPayload().id("process_1")
    );

    // Validate
    std::string error;
    EXPECT_FALSE(process_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "PROCESS_DELETE failed, process [process_1] doesn't exist.");
  }

  {  // Successful
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create the process action
    ProcessAction process_action(
      action_type::PROCESS_DELETE,
      ActionPayload().id("process_1")
    );

    // Validate
    std::string error;
    EXPECT_TRUE(process_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    process_action.apply();

    // Check the process is deleted
    EXPECT_FALSE(schedule_cache->has_process("process_1"));

    // Check the events' process id has changed
    EXPECT_FALSE(schedule_cache->get_event("process_event_1")->process_id.has_value());
    EXPECT_FALSE(schedule_cache->get_event("process_event_2")->process_id.has_value());
    EXPECT_FALSE(schedule_cache->get_event("process_event_3")->process_id.has_value());
    EXPECT_FALSE(schedule_cache->get_event("process_event_4")->process_id.has_value());
    EXPECT_FALSE(schedule_cache->get_event("process_event_5")->process_id.has_value());

    // Check record
    std::vector<ChangeAction> expected_event_record {
      {"process_event_1", "update"},
      {"process_event_2", "update"},
      {"process_event_3", "update"},
      {"process_event_4", "update"},
      {"process_event_5", "update"},
    };
    EXPECT_EQ(
      process_action.record().get("event"),
      expected_event_record
    );

    std::vector<ChangeAction> expected_process_record {
      {"process_1", "delete"},
    };

    EXPECT_EQ(
      process_action.record().get("process"),
      expected_process_record
    );
  }
}

TEST_F(TestCacheProcessAction, process_delete_all) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Successful
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache1();

    // Create the process action
    ProcessAction process_action(
      action_type::PROCESS_DELETE_ALL,
      ActionPayload().id("process_1")
    );

    // Validate
    std::string error;
    EXPECT_TRUE(process_action.validate(schedule_cache, error));
    EXPECT_TRUE(error.empty());

    // Apply
    process_action.apply();

    // Check the process is deleted
    EXPECT_FALSE(schedule_cache->has_process("process_1"));

    // Check the events' process id has changed
    EXPECT_FALSE(schedule_cache->has_event("process_event_1"));
    EXPECT_FALSE(schedule_cache->has_event("process_event_2"));
    EXPECT_FALSE(schedule_cache->has_event("process_event_3"));
    EXPECT_FALSE(schedule_cache->has_event("process_event_4"));
    EXPECT_FALSE(schedule_cache->has_event("process_event_5"));

    // Check record
    std::vector<ChangeAction> expected_event_record {
      {"process_event_1", "delete"},
      {"process_event_2", "delete"},
      {"process_event_3", "delete"},
      {"process_event_4", "delete"},
      {"process_event_5", "delete"},
    };
    EXPECT_EQ(
      process_action.record().get("event"),
      expected_event_record
    );

    std::vector<ChangeAction> expected_process_record {
      {"process_1", "delete"},
    };

    EXPECT_EQ(
      process_action.record().get("process"),
      expected_process_record
    );
  }
}
