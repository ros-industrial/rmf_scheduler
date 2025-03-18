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
#include "rmf2_scheduler/data/graph.hpp"
#include "rmf2_scheduler/utils/tree_converter.hpp"
#include "../../utils/gtest_macros.hpp"

namespace rmf2_scheduler
{
namespace data
{

Graph::UPtr make_preset_graph()
{
  Graph::UPtr graph = std::make_unique<Graph>();
  graph->add_node("node_0");
  graph->add_node("node_1");
  graph->add_node("node_2");
  graph->add_node("node_3");
  graph->add_node("node_4");
  graph->add_edge("node_0", "node_2");
  graph->add_edge("node_1", "node_2");
  graph->add_edge("node_2", "node_3");
  graph->add_edge("node_2", "node_4");
  return graph;
}


}  // namespace data
}  // namespace rmf2_scheduler

class TestUtilsTreeConverter : public ::testing::Test
{
protected:
  rmf2_scheduler::utils::TreeConversion converter;
};

TEST_F(TestUtilsTreeConverter, empty) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  // empty graph conversion test
  Graph empty;
  EXPECT_EQ(converter.convert_to_tree(empty), "");

  // test with only 1 node in the graph
  empty.add_node("node_1");
  std::string expected_tree =
    R"(<root BTCPP_format="4">
    <BehaviorTree ID="dag-numbers">
        <node_1/>
    </BehaviorTree>
</root>
)";
  EXPECT_EQ(expected_tree, converter.convert_to_tree(empty));
}

TEST_F(TestUtilsTreeConverter, basic_configuration) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  // basic sequence test
  Graph basic;

  basic.add_node("node_1");
  basic.add_node("node_2");
  basic.add_node("node_3");
  basic.add_edge("node_1", "node_2");
  basic.add_edge("node_2", "node_3");

  std::string expected_tree =
    R"(<root BTCPP_format="4">
    <BehaviorTree ID="dag-numbers">
        <Sequence>
            <node_1/>
            <node_2/>
            <node_3/>
        </Sequence>
    </BehaviorTree>
</root>
)";
  EXPECT_EQ(converter.convert_to_tree(basic), expected_tree);

  // basic parallel test
  basic.delete_node("node_3");
  basic.add_node("node_3");
  basic.add_edge("node_1", "node_3");
  expected_tree =
    R"(<root BTCPP_format="4">
    <BehaviorTree ID="dag-numbers">
        <Sequence>
            <node_1/>
            <Parallel>
                <node_3/>
                <node_2/>
            </Parallel>
        </Sequence>
    </BehaviorTree>
</root>
)";
  EXPECT_EQ(converter.convert_to_tree(basic), expected_tree);
}
TEST_F(TestUtilsTreeConverter, multiple_entry_nodes) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Graph::UPtr graph = make_preset_graph();
  // unconnected nodes test
  {
    graph->add_node("node_5");
    graph->add_node("node_6");
    EXPECT_THROW_EQ(
      std::string result = converter.convert_to_tree(*graph),
      std::logic_error("Graph contains unconnected node(s)! Cannot convert graph to tree!")
    );
    graph->prune();
  }
  // cyclic test
  {
    graph->add_edge("node_4", "node_0");
    EXPECT_THROW_EQ(
      std::string result = converter.convert_to_tree(*graph),
      std::logic_error("Cycle(s) detected in graph! Cannot convert graph to tree!")
    );
    graph->delete_edge("node_4", "node_0");
    EXPECT_NO_THROW(converter.convert_to_tree(*graph));
  }
  std::string expected_tree =
    R"(<root BTCPP_format="4">
    <BehaviorTree ID="dag-numbers">
        <Sequence>
            <Parallel>
                <node_1/>
                <node_0/>
            </Parallel>
            <Sequence>
                <node_2/>
                <Parallel>
                    <node_4/>
                    <node_3/>
                </Parallel>
            </Sequence>
        </Sequence>
    </BehaviorTree>
</root>
)";
  EXPECT_EQ(expected_tree, converter.convert_to_tree(*graph));

  // multiple entry with diamond configuration test
  graph->add_node("node_5");
  graph->add_edge("node_4", "node_5");
  graph->add_node("node_6");
  graph->add_node("node_7");
  graph->add_edge("node_5", "node_7");
  graph->add_edge("node_3", "node_6");
  graph->add_edge("node_6", "node_7");
  expected_tree =
    R"(<root BTCPP_format="4">
    <BehaviorTree ID="dag-numbers">
        <Sequence>
            <Parallel>
                <node_1/>
                <node_0/>
            </Parallel>
            <Sequence>
                <node_2/>
                <Parallel>
                    <Sequence>
                        <node_4/>
                        <node_5/>
                    </Sequence>
                    <Sequence>
                        <node_3/>
                        <node_6/>
                    </Sequence>
                </Parallel>
                <node_7/>
            </Sequence>
        </Sequence>
    </BehaviorTree>
</root>
)";
  EXPECT_EQ(expected_tree, converter.convert_to_tree(*graph));
}

TEST_F(TestUtilsTreeConverter, common_ancestors) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  // binary tree configuration test
  Graph::UPtr graph = make_preset_graph();
  graph->delete_node("node_1");
  graph->add_node("node_5");
  graph->add_edge("node_3", "node_5");
  graph->add_node("node_6");
  graph->add_edge("node_4", "node_6");
  graph->add_node("node_7");
  graph->add_edge("node_3", "node_7");
  graph->add_node("node_8");
  graph->add_edge("node_4", "node_8");
  std::string expected_tree =
    R"(<root BTCPP_format="4">
    <BehaviorTree ID="dag-numbers">
        <Sequence>
            <node_0/>
            <Sequence>
                <node_2/>
                <Parallel>
                    <Sequence>
                        <node_4/>
                        <Parallel>
                            <node_8/>
                            <node_6/>
                        </Parallel>
                    </Sequence>
                    <Sequence>
                        <node_3/>
                        <Parallel>
                            <node_7/>
                            <node_5/>
                        </Parallel>
                    </Sequence>
                </Parallel>
            </Sequence>
        </Sequence>
    </BehaviorTree>
</root>
)";
  EXPECT_EQ(expected_tree, converter.convert_to_tree(*graph));

  // multiple inbound nodes test
  Graph::UPtr complex_graph = std::make_unique<Graph>();
  complex_graph->add_node("node_1");
  complex_graph->add_node("node_2");
  complex_graph->add_node("node_3");
  complex_graph->add_node("node_4");
  complex_graph->add_node("node_5");
  complex_graph->add_node("node_6");
  complex_graph->add_node("node_7");
  complex_graph->add_edge("node_1", "node_2");
  complex_graph->add_edge("node_2", "node_3");
  complex_graph->add_edge("node_3", "node_4");
  complex_graph->add_edge("node_3", "node_5");
  complex_graph->add_edge("node_4", "node_7");
  complex_graph->add_edge("node_2", "node_6");
  complex_graph->add_edge("node_6", "node_7");
  complex_graph->add_edge("node_5", "node_7");
  expected_tree =
    R"(<root BTCPP_format="4">
    <BehaviorTree ID="dag-numbers">
        <Sequence>
            <node_1/>
            <Sequence>
                <node_2/>
                <Parallel>
                    <node_6/>
                    <Sequence>
                        <node_3/>
                        <Parallel>
                            <node_5/>
                            <node_4/>
                        </Parallel>
                    </Sequence>
                </Parallel>
            </Sequence>
            <node_7/>
        </Sequence>
    </BehaviorTree>
</root>
)";
  EXPECT_EQ(expected_tree, converter.convert_to_tree(*complex_graph));

  Graph::UPtr g_ = std::make_unique<Graph>();
  g_->add_node("node_1");
  g_->add_node("node_2");
  g_->add_node("node_3");
  g_->add_node("node_4");
  g_->add_node("node_5");
  g_->add_node("node_6");
  g_->add_node("node_7");
  g_->add_node("node_8");
  g_->add_edge("node_1", "node_2");
  g_->add_edge("node_1", "node_3");
  g_->add_edge("node_2", "node_4");
  g_->add_edge("node_2", "node_5");
  g_->add_edge("node_3", "node_6");
  g_->add_edge("node_3", "node_8");
  g_->add_edge("node_4", "node_7");
  g_->add_edge("node_5", "node_7");
  g_->add_edge("node_6", "node_7");
  g_->add_edge("node_8", "node_7");
  expected_tree =
    R"(<root BTCPP_format="4">
    <BehaviorTree ID="dag-numbers">
        <Sequence>
            <node_1/>
            <Parallel>
                <Sequence>
                    <node_3/>
                    <Parallel>
                        <node_8/>
                        <node_6/>
                    </Parallel>
                </Sequence>
                <Sequence>
                    <node_2/>
                    <Parallel>
                        <node_5/>
                        <node_4/>
                    </Parallel>
                </Sequence>
            </Parallel>
            <node_7/>
        </Sequence>
    </BehaviorTree>
</root>
)";
  EXPECT_EQ(expected_tree, converter.convert_to_tree(*g_));

  Graph::UPtr g = std::make_unique<Graph>();
  g->add_node("R1->Pickup 1");
  g->add_node("R1->Drop 1");
  g->add_node("R2->Pickup 1");

  g->add_edge("R1->Pickup 1", "R1->Drop 1");
  g->add_edge("R1->Pickup 1", "R2->Pickup 1");

  g->add_node("R3->Pickup 1");
  g->add_node("R2->Drop 1");
  g->add_node("R1->Resting 1");

  g->add_edge("R2->Pickup 1", "R2->Drop 1");
  g->add_edge("R2->Pickup 1", "R3->Pickup 1");
  g->add_edge("R1->Drop 1", "R1->Resting 1");
  g->add_edge("R1->Drop 1", "R2->Drop 1");

  g->add_node("R4->Pickup 1");
  g->add_node("R3->Drop 1");
  g->add_node("R2->Rest 2");

  g->add_edge("R2->Drop 1", "R2->Rest 2");
  g->add_edge("R2->Drop 1", "R3->Drop 1");
  g->add_edge("R3->Pickup 1", "R3->Drop 1");
  g->add_edge("R3->Pickup 1", "R4->Pickup 1");

  g->add_node("R4->Drop 1");
  g->add_node("R3->Rest 3");
  g->add_node("R4->Rest 4");

  g->add_edge("R4->Pickup 1", "R4->Drop 1");
  g->add_edge("R3->Drop 1", "R4->Drop 1");
  g->add_edge("R3->Drop 1", "R3->Rest 3");
  g->add_edge("R4->Drop 1", "R4->Rest 4");

  expected_tree =
    R"(<root BTCPP_format="4">
    <BehaviorTree ID="dag-numbers">
        <Sequence>
            <R1->Pickup 1/>
            <Parallel>
                <Sequence>
                    <R2->Pickup 1/>
                    <Sequence>
                        <R3->Pickup 1/>
                        <R4->Pickup 1/>
                    </Sequence>
                </Sequence>
                <Sequence>
                    <R1->Drop 1/>
                    <R1->Resting 1/>
                </Sequence>
            </Parallel>
            <Sequence>
                <R2->Drop 1/>
                <R2->Rest 2/>
            </Sequence>
            <Sequence>
                <R3->Drop 1/>
                <R3->Rest 3/>
            </Sequence>
            <R4->Drop 1/>
            <R4->Rest 4/>
        </Sequence>
    </BehaviorTree>
</root>
)";
  EXPECT_EQ(expected_tree, converter.convert_to_tree(*g));
}
