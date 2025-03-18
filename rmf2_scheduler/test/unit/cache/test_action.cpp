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

#include "rmf2_scheduler/cache/action.hpp"

class TestCacheActionPayload : public ::testing::Test
{
};

TEST_F(TestCacheActionPayload, payload_read_write) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Empty payload
    ActionPayload payload;

    auto data = payload.data(action_type::EVENT_ADD);
    ScheduleAction expected_data;
    expected_data.type = action_type::EVENT_ADD;
    EXPECT_EQ(data, expected_data);
  }

  {  // Full payload
    ActionPayload payload;

    // Payload to update
    std::string id_payload = "task_1";
    Event::Ptr event_payload = std::make_shared<Event>(
      "task_2",
      "task_type",
      Time(10 * 60)
    );
    Task::Ptr task_payload = std::make_shared<Task>(
      *event_payload,
      "ongoing"
    );

    Process::Ptr process_payload = std::make_shared<Process>();
    process_payload->id = "process_1";
    process_payload->graph.add_node("task_1");
    process_payload->graph.add_node("task_2");
    process_payload->graph.add_edge("task_1", "task_2");

    std::string node_id_payload = "task_1";
    std::string source_id_payload = "task_2";
    std::string destination_id_payload = "task_2";
    std::string edge_type_payload = "hard";


    // Write
    payload.id(id_payload);
    payload.event(event_payload);
    payload.task(task_payload);
    payload.process(process_payload);
    payload.node_id(node_id_payload);
    payload.source_id(source_id_payload);
    payload.destination_id(destination_id_payload);
    payload.edge_type(edge_type_payload);

    // Check value
    auto data = payload.data(action_type::EVENT_ADD);
    EXPECT_EQ(data.id.value(), id_payload);
    EXPECT_EQ(data.node_id.value(), node_id_payload);
    EXPECT_EQ(data.source_id.value(), source_id_payload);
    EXPECT_EQ(data.destination_id.value(), destination_id_payload);
    EXPECT_EQ(data.edge_type.value(), edge_type_payload);

    EXPECT_NE(data.event, event_payload);  // check deep copy
    EXPECT_EQ(*data.event, *event_payload);
    EXPECT_NE(data.task, task_payload);  // check deep copy
    EXPECT_EQ(*data.task, *task_payload);
    EXPECT_NE(data.process, process_payload);  // check deep copy
    EXPECT_EQ(*data.process, *process_payload);
  }

  {  // Builder pattern
    // Payload to update
    std::string id_payload = "task_1";
    Event::Ptr event_payload = std::make_shared<Event>(
      "task_2",
      "task_type",
      Time(10 * 60)
    );
    Task::Ptr task_payload = std::make_shared<Task>(
      *event_payload,
      "ongoing"
    );

    Process::Ptr process_payload = std::make_shared<Process>();
    process_payload->id = "process_1";
    process_payload->graph.add_node("task_1");
    process_payload->graph.add_node("task_2");
    process_payload->graph.add_edge("task_1", "task_2");

    std::string node_id_payload = "task_1";
    std::string source_id_payload = "task_2";
    std::string destination_id_payload = "task_2";
    std::string edge_type_payload = "hard";


    // Write
    auto payload = ActionPayload()
      .id(id_payload)
      .event(event_payload)
      .task(task_payload)
      .process(process_payload)
      .node_id(node_id_payload)
      .source_id(source_id_payload)
      .destination_id(destination_id_payload)
      .edge_type(edge_type_payload);

    // Check value
    auto data = payload.data(action_type::EVENT_ADD);
    EXPECT_EQ(data.id.value(), id_payload);
    EXPECT_EQ(data.node_id.value(), node_id_payload);
    EXPECT_EQ(data.source_id.value(), source_id_payload);
    EXPECT_EQ(data.destination_id.value(), destination_id_payload);
    EXPECT_EQ(data.edge_type.value(), edge_type_payload);

    EXPECT_NE(data.event, event_payload);  // check deep copy
    EXPECT_EQ(*data.event, *event_payload);
    EXPECT_NE(data.task, task_payload);  // check deep copy
    EXPECT_EQ(*data.task, *task_payload);
    EXPECT_NE(data.process, process_payload);  // check deep copy
    EXPECT_EQ(*data.process, *process_payload);
  }
}
