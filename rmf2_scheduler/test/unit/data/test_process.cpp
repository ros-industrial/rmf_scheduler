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

#include "rmf2_scheduler/data/process.hpp"

class TestDataProcess : public ::testing::Test
{
};

TEST_F(TestDataProcess, equal) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Process process;
  process.id = "13aa1c62-64ca-495d-a4b7-84de6a00f56a";
  process.graph.add_node("node_0");
  process.graph.add_node("node_1");
  process.graph.add_node("node_2");
  process.graph.add_node("node_3");
  process.graph.add_node("node_4");
  process.graph.add_edge("node_0", "node_2", Edge("hard"));
  process.graph.add_edge("node_1", "node_2", Edge("soft"));
  process.graph.add_edge("node_2", "node_3", Edge("hard"));
  process.graph.add_edge("node_2", "node_4", Edge("soft"));

  {  // Equal
    Process process_to_compare;
    process_to_compare.id = "13aa1c62-64ca-495d-a4b7-84de6a00f56a";
    process_to_compare.graph.add_node("node_0");
    process_to_compare.graph.add_node("node_1");
    process_to_compare.graph.add_node("node_2");
    process_to_compare.graph.add_node("node_3");
    process_to_compare.graph.add_node("node_4");
    process_to_compare.graph.add_edge("node_0", "node_2", Edge("hard"));
    process_to_compare.graph.add_edge("node_1", "node_2", Edge("soft"));
    process_to_compare.graph.add_edge("node_2", "node_3", Edge("hard"));
    process_to_compare.graph.add_edge("node_2", "node_4", Edge("soft"));
    EXPECT_EQ(process, process_to_compare);
  }

  {  // Different id
    Process process_to_compare;
    process_to_compare.id = "123456";
    process_to_compare.graph.add_node("node_0");
    process_to_compare.graph.add_node("node_1");
    process_to_compare.graph.add_node("node_2");
    process_to_compare.graph.add_node("node_3");
    process_to_compare.graph.add_node("node_4");
    process_to_compare.graph.add_edge("node_0", "node_2", Edge("hard"));
    process_to_compare.graph.add_edge("node_1", "node_2", Edge("soft"));
    process_to_compare.graph.add_edge("node_2", "node_3", Edge("hard"));
    process_to_compare.graph.add_edge("node_2", "node_4", Edge("soft"));
    EXPECT_NE(process, process_to_compare);
  }

  {  // Different graph
    Process process_to_compare;
    process_to_compare.id = "123456";
    process_to_compare.graph.add_node("node_0");
    process_to_compare.graph.add_node("node_1");
    process_to_compare.graph.add_node("node_2");
    process_to_compare.graph.add_edge("node_0", "node_2", Edge("hard"));
    process_to_compare.graph.add_edge("node_1", "node_2", Edge("soft"));
    EXPECT_NE(process, process_to_compare);
  }
}
