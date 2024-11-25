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

#include "rmf2_scheduler/data/time.hpp"
#include "rmf2_scheduler/data/duration.hpp"
#include "rmf2_scheduler/data/event.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"
#include "../../utils/gtest_macros.hpp"


class TestJSONSerializer : public ::testing::Test
{
};

TEST_F(TestJSONSerializer, time) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using json = nlohmann::json;
  {
    // Serialization
    EXPECT_EQ(R"(1.0)"_json, json(Time(1, 0)));
    EXPECT_EQ(R"(4.5)"_json, json(Time(4, 500000000)));
    EXPECT_EQ(R"(2.5)"_json, json(Time(0, 2500000000)));
  }

  {
    // Deserialization
    EXPECT_EQ(R"(1.0)"_json.template get<Time>().nanoseconds(), Time(1, 0).nanoseconds());
    EXPECT_EQ(R"(4.5)"_json.template get<Time>().nanoseconds(), Time(4, 500000000).nanoseconds());
    EXPECT_EQ(R"(2.5)"_json.template get<Time>().nanoseconds(), Time(0, 2500000000).nanoseconds());
  }
}

TEST_F(TestJSONSerializer, duration) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using json = nlohmann::json;
  {
    // Serialization
    EXPECT_EQ(R"(1.0)"_json, json(Duration(1, 0)));
    EXPECT_EQ(R"(4.5)"_json, json(Duration(4, 500000000)));
    EXPECT_EQ(R"(2.5)"_json, json(Duration(0, 2500000000)));
  }

  {
    // Deserialization
    EXPECT_EQ(
      R"(1.0)"_json.template get<Duration>().nanoseconds(),
      Duration(1, 0).nanoseconds()
    );
    EXPECT_EQ(
      R"(4.5)"_json.template get<Duration>().nanoseconds(),
      Duration(4, 500000000).nanoseconds()
    );
    EXPECT_EQ(
      R"(2.5)"_json.template get<Duration>().nanoseconds(),
      Duration(0, 2500000000).nanoseconds()
    );
  }
}

TEST_F(TestJSONSerializer, graph) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using json = nlohmann::json;

  json graph_json =
    R"([
    { "id": "node_0", "needs": [] },
    { "id": "node_1", "needs": [] },
    { "id": "node_2", "needs": [
      { "id": "node_0", "type": "hard" },
      { "id": "node_1", "type": "soft" }
    ] },
    { "id": "node_3", "needs": [{ "id": "node_2", "type": "hard"}] },
    { "id": "node_4", "needs": [{ "id": "node_2", "type": "soft"}] }
  ])"_json;

  Graph graph;
  graph.add_node("node_0");
  graph.add_node("node_1");
  graph.add_node("node_2");
  graph.add_node("node_3");
  graph.add_node("node_4");
  graph.add_edge("node_0", "node_2", Edge("hard"));
  graph.add_edge("node_1", "node_2", Edge("soft"));
  graph.add_edge("node_2", "node_3", Edge("hard"));
  graph.add_edge("node_2", "node_4", Edge("soft"));

  // Serialization
  EXPECT_EQ(graph_json, json(graph));

  // Deserialization
  EXPECT_EQ(graph, graph_json.template get<Graph>());
}

TEST_F(TestJSONSerializer, event) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using json = nlohmann::json;

  {
    // Empty event
    Event empty_event;
    json empty_event_json =
      R"({
      "id": "",
      "type": "",
      "description": null,
      "start_time": 0.0,
      "end_time": null,
      "series_id": null,
      "process_id": null
    })"_json;

    // Serialization
    EXPECT_EQ(
      empty_event_json,
      json(empty_event)
    );
  }

  {
    // Minimal event
    Event minimal_event(
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "event_type",
      Time(1672625700, 0)
    );
    json minimal_event_json =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "event_type",
      "description": null,
      "start_time": 1672625700.0,
      "end_time": null,
      "series_id": null,
      "process_id": null
    })"_json;

    json minimal_event_json_wo_null =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "event_type",
      "start_time": 1672625700.0
    })"_json;

    // Serialization
    EXPECT_EQ(
      minimal_event_json,
      json(minimal_event)
    );

    // Deserialization
    EXPECT_EQ(
      minimal_event,
      minimal_event_json.template get<Event>()
    );
    EXPECT_EQ(
      minimal_event,
      minimal_event_json_wo_null.template get<Event>()
    );
  }

  {
    // Full event
    Event full_event {
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "event_type",
      "This is a event!",
      Time(1672625700, 0),
      Duration(3600, 0),
      "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "13aa1c62-64ca-495d-a4b7-84de6a00f56a"
    };

    json full_event_json =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "event_type",
      "description": "This is a event!",
      "start_time": 1672625700.0,
      "end_time": 1672629300.0,
      "series_id": "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a"
    })"_json;

    // Serialization
    EXPECT_EQ(
      full_event_json,
      json(full_event)
    );

    // Deserialization
    EXPECT_EQ(
      full_event,
      full_event_json.template get<Event>()
    );
  }

  {
    // Invalid end time
    json full_event_json =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "event_type",
      "description": "This is a event!",
      "start_time": 1672625700.0,
      "end_time": 1672624700.0,
      "series_id": "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a"
    })"_json;

    // Expect throw
    EXPECT_THROW_EQ(
      full_event_json.template get<Event>(),
      std::underflow_error(
        "from_json failed, event end_time is before start_time."
      )
    );
  }
}

TEST_F(TestJSONSerializer, task) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using json = nlohmann::json;

  {
    // Empty task
    Task empty_task;
    json empty_task_json =
      R"({
      "id": "",
      "type": "",
      "description": null,
      "start_time": 0.0,
      "end_time": null,
      "series_id": null,
      "process_id": null,
      "resource_id": null,
      "deadline": null,
      "status": "",
      "planned_start_time": null,
      "planned_end_time": null,
      "estimated_duration": null,
      "actual_start_time": null,
      "actual_end_time": null,
      "task_details": null
    })"_json;

    // Serialization
    EXPECT_EQ(
      empty_task_json,
      json(empty_task)
    );
  }

  {
    // Minimal task
    Task minimal_task(
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "task_type",
      Time(1672625700, 0),
      "draft"
    );
    json minimal_task_json =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "task_type",
      "description": null,
      "start_time": 1672625700.0,
      "end_time": null,
      "series_id": null,
      "process_id": null,
      "resource_id": null,
      "deadline": null,
      "status": "draft",
      "planned_start_time": null,
      "planned_end_time": null,
      "estimated_duration": null,
      "actual_start_time": null,
      "actual_end_time": null,
      "task_details": null
    })"_json;

    json minimal_task_json_wo_null =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "task_type",
      "start_time": 1672625700.0,
      "status": "draft"
    })"_json;

    // Serialization
    EXPECT_EQ(
      minimal_task_json,
      json(minimal_task)
    );

    // Deserialization
    EXPECT_EQ(
      minimal_task,
      minimal_task_json.template get<Task>()
    );
    EXPECT_EQ(
      minimal_task,
      minimal_task_json_wo_null.template get<Task>()
    );
  }

  {
    // Full task
    Task full_task {
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
      R"({"some_field": "some_value"})"_json,
    };

    json full_task_json =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "task_type",
      "description": "This is a task!",
      "start_time": 1672625700.0,
      "end_time": 1672629300.0,
      "series_id": "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
      "resource_id": "09d5323d-43e0-4668-b53a-3cf33f9b9a96",
      "deadline": 1672632900.0,
      "status": "ongoing",
      "planned_start_time": 1672625700.0,
      "planned_end_time": 1672629300.0,
      "estimated_duration": 3700.0,
      "actual_start_time": 1672625760.0,
      "actual_end_time": 1672629520.0,
      "task_details": {
        "some_field": "some_value"
      }
    })"_json;

    // Serialization
    EXPECT_EQ(
      full_task_json,
      json(full_task)
    );

    // Deserialization
    EXPECT_EQ(
      full_task,
      full_task_json.template get<Task>()
    );
  }

  {
    // Invalid planned_start_time
    json task_json =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "task_type",
      "start_time": 1672625700.0,
      "status": "draft",
      "planned_start_time": null,
      "planned_end_time": 1672629300.0
    })"_json;

    // Expect throw
    EXPECT_THROW_EQ(
      task_json.template get<Task>(),
      std::underflow_error(
        "from_json failed, task planned_end_time is defined but planned_start_time is null."
      )
    );
  }

  {
    // Invalid planned_end_time
    json task_json =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "task_type",
      "start_time": 1672625700.0,
      "status": "draft",
      "planned_start_time": 1672625700.0,
      "planned_end_time": 1672624700.0
    })"_json;

    // Expect throw
    EXPECT_THROW_EQ(
      task_json.template get<Task>(),
      std::underflow_error(
        "from_json failed, task planned_end_time is before planned_start_time."
      )
    );
  }

  {
    // Invalid actual_start_time
    json task_json =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "task_type",
      "start_time": 1672625700.0,
      "status": "draft",
      "actual_start_time": null,
      "actual_end_time": 1672629520.0
    })"_json;

    // Expect throw
    EXPECT_THROW_EQ(
      task_json.template get<Task>(),
      std::underflow_error(
        "from_json failed, task actual_end_time is defined but actual_start_time is null."
      )
    );
  }

  {
    // Invalid actual_end_time
    json task_json =
      R"({
      "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "type": "task_type",
      "start_time": 1672625700.0,
      "status": "draft",
      "actual_start_time": 1672625760.0,
      "actual_end_time": 1672624760.0
    })"_json;

    // Expect throw
    EXPECT_THROW_EQ(
      task_json.template get<Task>(),
      std::underflow_error(
        "from_json failed, task actual_end_time is before actual_start_time."
      )
    );
  }
}

TEST_F(TestJSONSerializer, process) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using json = nlohmann::json;

  {
    // Empty process
    Process empty_process;
    json empty_process_json =
      R"({
      "id": "",
      "graph": []
    })"_json;

    // Serialize empty process
    EXPECT_EQ(
      empty_process_json,
      json(empty_process));
  }

  {
    // Full process
    Process full_process;
    full_process.id = "13aa1c62-64ca-495d-a4b7-84de6a00f56a";
    full_process.graph.add_node("node_0");
    full_process.graph.add_node("node_1");
    full_process.graph.add_node("node_2");
    full_process.graph.add_node("node_3");
    full_process.graph.add_node("node_4");
    full_process.graph.add_edge("node_0", "node_2", Edge("hard"));
    full_process.graph.add_edge("node_1", "node_2", Edge("soft"));
    full_process.graph.add_edge("node_2", "node_3", Edge("hard"));
    full_process.graph.add_edge("node_2", "node_4", Edge("soft"));

    json full_process_json =
      R"({
      "id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
      "graph": [
        { "id": "node_0", "needs": [] },
        { "id": "node_1", "needs": [] },
        { "id": "node_2", "needs": [
          { "id": "node_0", "type": "hard" },
          { "id": "node_1", "type": "soft" }
        ] },
        { "id": "node_3", "needs": [{ "id": "node_2", "type": "hard"}] },
        { "id": "node_4", "needs": [{ "id": "node_2", "type": "soft"}] }
      ]
    })"_json;

    // Serialize full process
    EXPECT_EQ(
      full_process_json,
      json(full_process)
    );

    // Deserialize full process
    EXPECT_EQ(
      full_process,
      full_process_json.template get<Process>()
    );
  }
}

TEST_F(TestJSONSerializer, occurrence) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using json = nlohmann::json;

  Occurrence occurrence {
    Time(1672625700.0, 0),
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"
  };

  json occurrence_json =
    R"({
    "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "time": 1672625700.0
  })"_json;

  // Serialization
  EXPECT_EQ(occurrence_json, json(occurrence));

  // Deserialization
  EXPECT_EQ(occurrence, occurrence_json.template get<Occurrence>());
}

TEST_F(TestJSONSerializer, series) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using json = nlohmann::json;

  {  // Minimal Series
    Series series(
      "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "task",
    {
      Occurrence{Time(1672625700, 0), "24285fa4-496b-4721-9b16-536f8ff25378"},
      Occurrence{Time(1672712100, 0), "b10f0a98-ee4a-4efd-84a8-b5f4cb02a936"},
      Occurrence{Time(1672798500, 0), "ccc7800a-af59-4eec-ac61-510ac9bd1d6f"},
      Occurrence{Time(1672884900, 0), "01a882f5-32e8-4245-9b4d-128ef3c783d2"},
      Occurrence{Time(1672971300, 0), "c66e12c9-5c8b-4e78-b562-8917d8e1c99e"},
    },
      "0 15 10 ? * MON-FRI",    // 10:15 AM every Monday - Friday
      "Asia/Singapore"
    );

    json series_json =
      R"({
      "id": "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "type": "task",
      "occurrences":[
        {"id": "24285fa4-496b-4721-9b16-536f8ff25378", "time": 1672625700.0},
        {"id": "b10f0a98-ee4a-4efd-84a8-b5f4cb02a936", "time": 1672712100.0},
        {"id": "ccc7800a-af59-4eec-ac61-510ac9bd1d6f", "time": 1672798500.0},
        {"id": "01a882f5-32e8-4245-9b4d-128ef3c783d2", "time": 1672884900.0},
        {"id": "c66e12c9-5c8b-4e78-b562-8917d8e1c99e", "time": 1672971300.0}
      ],
      "cron": "0 15 10 ? * MON-FRI",
      "timezone": "Asia/Singapore"
    })"_json;

    // Serialization
    EXPECT_EQ(series_json, json(series));

    // Deserialization
    EXPECT_EQ(series, series_json.template get<Series>());
  }

  {  // Full Series
    Series series(
      "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "task",
    {
      Occurrence{Time(1672625700, 0), "24285fa4-496b-4721-9b16-536f8ff25378"},
      Occurrence{Time(1672720682, 0), "b10f0a98-ee4a-4efd-84a8-b5f4cb02a936"},
      Occurrence{Time(1672798500, 0), "ccc7800a-af59-4eec-ac61-510ac9bd1d6f"},
      Occurrence{Time(1672884900, 0), "01a882f5-32e8-4245-9b4d-128ef3c783d2"},
      Occurrence{Time(1672971300, 0), "c66e12c9-5c8b-4e78-b562-8917d8e1c99e"},
    },
      "0 15 10 ? * MON-FRI",    // 10:15 AM every Monday - Friday
      "Asia/Singapore",
      Time(1696299300, 0),
      {"b10f0a98-ee4a-4efd-84a8-b5f4cb02a936"}
    );

    json series_json =
      R"({
      "id": "7c98b392-2131-4528-92d2-7e7f22d0a9a5",
      "type": "task",
      "occurrences":[
        {"id": "24285fa4-496b-4721-9b16-536f8ff25378", "time": 1672625700.0},
        {"id": "b10f0a98-ee4a-4efd-84a8-b5f4cb02a936", "time": 1672720682.0},
        {"id": "ccc7800a-af59-4eec-ac61-510ac9bd1d6f", "time": 1672798500.0},
        {"id": "01a882f5-32e8-4245-9b4d-128ef3c783d2", "time": 1672884900.0},
        {"id": "c66e12c9-5c8b-4e78-b562-8917d8e1c99e", "time": 1672971300.0}
      ],
      "cron": "0 15 10 ? * MON-FRI",
      "timezone": "Asia/Singapore",
      "until": 1696299300.0,
      "exception_ids": [
        "b10f0a98-ee4a-4efd-84a8-b5f4cb02a936"
      ]
    })"_json;

    // Serialization
    EXPECT_EQ(series_json, json(series));

    // Deserialization
    EXPECT_EQ(series, series_json.template get<Series>());
  }
}
