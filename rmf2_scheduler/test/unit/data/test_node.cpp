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

#include "rmf2_scheduler/data/node.hpp"

namespace rmf2_scheduler
{

namespace data
{

class TestNode
{
public:
  static Node::UPtr make_preset_node()
  {
    auto node = std::make_unique<Node>("node_2");
    Node::Restricted token;
    node->inbound_edges(token).emplace("node_0", Edge("hard"));
    node->inbound_edges(token).emplace("node_1", Edge("soft"));
    node->outbound_edges(token).emplace("node_3", Edge("hard"));
    node->outbound_edges(token).emplace("node_4", Edge("soft"));
    return node;
  }

  static std::unordered_map<std::string, Edge> & inbound_edges_non_const(
    Node::Ptr node
  )
  {
    Node::Restricted token;
    return node->inbound_edges(token);
  }

  static std::unordered_map<std::string, Edge> & outbound_edges_non_const(
    Node::Ptr node
  )
  {
    Node::Restricted token;
    return node->outbound_edges(token);
  }
};

}  // namespace data
}  // namespace rmf2_scheduler

class TestDataNode : public ::testing::Test
{
};

TEST_F(TestDataNode, read_only) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Empty Node
    Node::UPtr node = std::make_unique<Node>("node_1");
    EXPECT_EQ("node_1", node->id());
    EXPECT_TRUE(node->inbound_edges().empty());
    EXPECT_TRUE(node->outbound_edges().empty());
  }

  {  // Preset Node
    Node::UPtr node = TestNode::make_preset_node();
    EXPECT_EQ("node_2", node->id());

    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_0", Edge("hard")},
      {"node_1", Edge("soft")},
    };
    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_3", Edge("hard")},
      {"node_4", Edge("soft")},
    };
    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }
}


TEST_F(TestDataNode, write) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {
    // Modify inbound & outbound edges
    Node::Ptr node = TestNode::make_preset_node();
    TestNode::inbound_edges_non_const(node).emplace("node_5", Edge("hard"));
    TestNode::outbound_edges_non_const(node).emplace("node_6", Edge("soft"));

    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_0", Edge("hard")},
      {"node_1", Edge("soft")},
      {"node_5", Edge("hard")},
    };
    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_3", Edge("hard")},
      {"node_4", Edge("soft")},
      {"node_6", Edge("soft")},
    };
    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }
}

TEST_F(TestDataNode, equal) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Node::Ptr node2 = TestNode::make_preset_node();
  Node::Ptr node2_duplicate = TestNode::make_preset_node();

  // different ID
  Node node3 = Node("node_3");

  // inbound edges different size
  Node::Ptr node2_empty_inbound_edges = TestNode::make_preset_node();
  TestNode::inbound_edges_non_const(node2_empty_inbound_edges).clear();

  // inbound edges different element
  Node::Ptr node2_different_inbound_edges = TestNode::make_preset_node();
  TestNode::inbound_edges_non_const(node2_different_inbound_edges).erase("node_0");
  TestNode::inbound_edges_non_const(node2_different_inbound_edges).emplace("node_5", Edge());

  // outbound edges different size
  Node::Ptr node2_empty_outbound_edges = TestNode::make_preset_node();
  TestNode::outbound_edges_non_const(node2_empty_outbound_edges).clear();

  // outbound edges different element
  Node::Ptr node2_different_outbound_edges = TestNode::make_preset_node();
  TestNode::outbound_edges_non_const(node2_different_outbound_edges).erase("node_0");
  TestNode::outbound_edges_non_const(node2_different_outbound_edges).emplace("node_5", Edge());

  EXPECT_NE(node2, node2_duplicate);
  EXPECT_TRUE(*node2 == *node2_duplicate);
  EXPECT_FALSE(*node2 != *node2_duplicate);
  EXPECT_TRUE(*node2 != node3);
  EXPECT_TRUE(*node2 != *node2_empty_inbound_edges);
  EXPECT_TRUE(*node2 != *node2_different_inbound_edges);
  EXPECT_TRUE(*node2 != *node2_empty_outbound_edges);
  EXPECT_TRUE(*node2 != *node2_different_outbound_edges);
}
