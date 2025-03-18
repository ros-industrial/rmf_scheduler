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

#include "rmf2_scheduler/data/json_serializer.hpp"
#include "rmf2_scheduler/data/graph.hpp"
#include "rmf2_scheduler/utils/dag_helper.hpp"

namespace rmf2_scheduler
{
namespace data
{

Graph::UPtr make_preset_graph()
{
  Graph::UPtr graph = std::make_unique<Graph>();
  graph->add_node("event_0");
  graph->add_node("event_1");
  graph->add_node("event_2");
  graph->add_node("event_3");
  graph->add_node("event_4");
  graph->add_edge("event_0", "event_2", Edge("hard"));
  graph->add_edge("event_1", "event_2", Edge("soft"));
  graph->add_edge("event_2", "event_3", Edge("hard"));
  graph->add_edge("event_2", "event_4", Edge("soft"));
  return graph;
}

std::unordered_map<std::string, data::Event::Ptr> make_preset_events()
{
  Event::Ptr event_0 = std::make_shared<Event>();
  event_0->id = "event_0";
  event_0->start_time = Time::from_localtime("Jan 2 10:15:00 2023");
  event_0->duration = std::chrono::minutes(10);

  Event::Ptr event_1 = std::make_shared<Event>();
  event_1->id = "event_1";
  event_1->start_time = Time::from_localtime("Jan 2 10:10:00 2023");
  event_1->duration = std::chrono::minutes(20);

  Event::Ptr event_2 = std::make_shared<Event>();
  event_2->id = "event_2";
  event_2->start_time = Time::from_localtime("Jan 2 10:10:00 2023");
  event_2->duration = std::chrono::minutes(30);

  Event::Ptr event_3 = std::make_shared<Event>();
  event_3->id = "event_3";
  event_3->start_time = Time::from_localtime("Jan 2 12:00:00 2023");
  event_3->duration = std::chrono::minutes(3);

  Event::Ptr event_4 = std::make_shared<Event>();
  event_4->id = "event_4";
  event_4->start_time = Time::from_localtime("Jan 2 11:10:00 2023");
  event_4->duration = std::chrono::minutes(5);
  return {
    {"event_0", event_0},
    {"event_1", event_1},
    {"event_2", event_2},
    {"event_3", event_3},
    {"event_4", event_4}
  };
}

}  // namespace data
}  // namespace rmf2_scheduler

class TestUtilsDAGHelper : public ::testing::Test
{
};

TEST_F(TestUtilsDAGHelper, is_valid_dag) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::utils;  // NOLINT(build/namespaces)

  {  // Valid DAG
    Graph::Ptr graph = make_preset_graph();
    EXPECT_TRUE(is_valid_dag(*graph));
  }

  {  // dangling node
    Graph::Ptr graph = make_preset_graph();
    graph->add_node("event_5");
    EXPECT_TRUE(is_valid_dag(*graph));
  }

  {  // Cyclic
    Graph::Ptr graph = make_preset_graph();
    graph->add_edge("event_4", "event_0");
    EXPECT_FALSE(is_valid_dag(*graph));
  }
}

TEST_F(TestUtilsDAGHelper, update_dag_start_time) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::utils;  // NOLINT(build/namespaces)

  Graph::ConstPtr graph = make_preset_graph();
  auto event_map = make_preset_events();

  update_dag_start_time(*graph, event_map);

  EXPECT_EQ(event_map["event_2"]->start_time, Time::from_localtime("Jan 2 10:30:00 2023"));
  EXPECT_EQ(event_map["event_3"]->start_time, Time::from_localtime("Jan 2 11:00:00 2023"));
  EXPECT_EQ(event_map["event_4"]->start_time, Time::from_localtime("Jan 2 11:10:00 2023"));
}
