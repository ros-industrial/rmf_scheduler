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
  graph->add_edge("node_0", "node_2", Edge("hard"));
  graph->add_edge("node_1", "node_2", Edge("soft"));
  graph->add_edge("node_2", "node_3", Edge("hard"));
  graph->add_edge("node_2", "node_4", Edge("soft"));
  return graph;
}


}  // namespace data
}  // namespace rmf2_scheduler

class TestDataGraph : public ::testing::Test
{
};

TEST_F(TestDataGraph, empty) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // empty
    Graph::ConstPtr graph = std::make_unique<Graph>();
    EXPECT_TRUE(graph->empty());
  }

  {  // not empty
    Graph::ConstPtr graph = make_preset_graph();
    EXPECT_FALSE(graph->empty());
  }
}

TEST_F(TestDataGraph, has_node) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Graph::ConstPtr graph = make_preset_graph();

  // has_node
  EXPECT_TRUE(graph->has_node("node_0"));
  EXPECT_TRUE(graph->has_node("node_1"));
  EXPECT_TRUE(graph->has_node("node_2"));
  EXPECT_TRUE(graph->has_node("node_3"));
  EXPECT_TRUE(graph->has_node("node_4"));
  EXPECT_FALSE(graph->has_node("node_5"));
}

TEST_F(TestDataGraph, get_node) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Graph::ConstPtr graph = make_preset_graph();
  {
    // get_node
    Node::ConstPtr node_0 = graph->get_node("node_0");
    Node::ConstPtr node_1 = graph->get_node("node_1");
    Node::ConstPtr node_2 = graph->get_node("node_2");
    Node::ConstPtr node_3 = graph->get_node("node_3");
    Node::ConstPtr node_4 = graph->get_node("node_4");
    EXPECT_THROW_EQ(
      graph->get_node("node_5"),
      std::out_of_range("Graph get_node failed, node [node_5] doesn't exist.")
    );

    // get_all_node
    std::unordered_map<std::string, Node::ConstPtr> expected_all_nodes {
      {"node_0", node_0},
      {"node_1", node_1},
      {"node_2", node_2},
      {"node_3", node_3},
      {"node_4", node_4},
    };

    EXPECT_EQ(expected_all_nodes, graph->get_all_nodes());

    // get_all_node_ordered
    std::map<std::string, Node::ConstPtr> expected_all_nodes_ordered {
      {"node_0", node_0},
      {"node_1", node_1},
      {"node_2", node_2},
      {"node_3", node_3},
      {"node_4", node_4},
    };

    EXPECT_EQ(expected_all_nodes_ordered, graph->get_all_nodes_ordered());
  }

  // Check preset node information
  {
    // node 0
    std::string node_id = "node_0";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_2", Edge("hard")}
    };

    // get_node
    Node::ConstPtr node = graph->get_node(node_id);

    // check id
    EXPECT_EQ(node_id, node->id());

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }

  {
    // node 1
    std::string node_id = "node_1";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_2", Edge("soft")}
    };

    // get_node
    Node::ConstPtr node = graph->get_node(node_id);

    // check id
    EXPECT_EQ(node_id, node->id());

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }

  {
    // node 2
    std::string node_id = "node_2";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_0", Edge("hard")},
      {"node_1", Edge("soft")},
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_3", Edge("hard")},
      {"node_4", Edge("soft")},
    };

    // get_node
    Node::ConstPtr node = graph->get_node(node_id);

    // check id
    EXPECT_EQ(node_id, node->id());

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }

  {
    // node 3
    std::string node_id = "node_3";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_2", Edge("hard")}
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
    };

    // get_node
    Node::ConstPtr node = graph->get_node(node_id);

    // check id
    EXPECT_EQ(node_id, node->id());

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }

  {
    // node 4
    std::string node_id = "node_4";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_2", Edge("soft")}
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
    };

    // get_node
    Node::ConstPtr node = graph->get_node(node_id);

    // check id
    EXPECT_EQ(node_id, node->id());

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }
}


TEST_F(TestDataGraph, dump) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Graph::ConstPtr graph = make_preset_graph();

  std::ostringstream oss;
  graph->dump(oss);
  auto dot = oss.str();

  std::string expected_dot =
    R"(digraph DAG {
node_0 -> node_2 [label=hard];
node_1 -> node_2 [label=soft];
node_2 -> node_3 [label=hard];
node_2 -> node_4 [label=soft];
}
)";

  EXPECT_EQ(expected_dot, dot);
}

TEST_F(TestDataGraph, add_node) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Graph::Ptr graph = make_preset_graph();
  EXPECT_THROW_EQ(
    graph->add_node("node_4"),
    std::invalid_argument("Graph add_node failed, node [node_4] already exists.")
  );

  graph->add_node("node_5");

  // New node has no edges
  EXPECT_TRUE(graph->get_node("node_5")->inbound_edges().empty());
  EXPECT_TRUE(graph->get_node("node_5")->outbound_edges().empty());
}

TEST_F(TestDataGraph, add_edge) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Graph::Ptr graph = make_preset_graph();

  EXPECT_THROW_EQ(
    graph->add_edge("node_5", "node_1"),
    std::out_of_range("Graph add_edge failed, node [node_5] doesn't exist.")
  );

  EXPECT_THROW_EQ(
    graph->add_edge("node_0", "node_5"),
    std::out_of_range("Graph add_edge failed, node [node_5] doesn't exist.")
  );

  EXPECT_THROW_EQ(
    graph->add_edge("node_0", "node_2"),
    std::invalid_argument("Graph add_edge failed, edge [node_0] to [node_2] already exists.")
  );

  graph->add_edge("node_0", "node_1");

  // Check edge exist
  {
    // node 0
    std::string node_id = "node_0";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_1", Edge("hard")},
      {"node_2", Edge("hard")},
    };

    Node::ConstPtr node = graph->get_node(node_id);

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }

  {
    // node 1
    std::string node_id = "node_1";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_0", Edge("hard")},
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_2", Edge("soft")},
    };

    Node::ConstPtr node = graph->get_node(node_id);

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }
}

TEST_F(TestDataGraph, update_node) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Graph::Ptr graph = make_preset_graph();

  EXPECT_THROW_EQ(
    graph->update_node("node_2", "node_2"),
    std::invalid_argument("Graph update_node failed, the new ID is the same as the old ID.")
  );

  EXPECT_THROW_EQ(
    graph->update_node("node_5", "node_2_new"),
    std::out_of_range("Graph update_node failed, node [node_5] doesn't exist.")
  );
  EXPECT_THROW_EQ(
    graph->update_node("node_2", "node_0"),
    std::invalid_argument("Graph update_node failed, new node ID [node_0] already exists.")
  );

  graph->update_node("node_2", "node_2_new");

  // Check all nodes are updated
  {
    // node 0
    std::string node_id = "node_0";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_2_new", Edge("hard")},
    };

    // get_node
    Node::ConstPtr node = graph->get_node(node_id);

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }
  {
    // node 1
    std::string node_id = "node_1";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_2_new", Edge("soft")},
    };

    Node::ConstPtr node = graph->get_node(node_id);

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }

  {
    // node 2 new
    std::string node_id = "node_2_new";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_0", Edge("hard")},
      {"node_1", Edge("soft")},
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_3", Edge("hard")},
      {"node_4", Edge("soft")},
    };

    Node::ConstPtr node = graph->get_node(node_id);

    // check id
    EXPECT_EQ(node_id, node->id());

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }

  {
    // node 3
    std::string node_id = "node_3";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_2_new", Edge("hard")}
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
    };

    Node::ConstPtr node = graph->get_node(node_id);

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }

  {
    // node 4
    std::string node_id = "node_4";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_2_new", Edge("soft")}
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
    };

    Node::ConstPtr node = graph->get_node(node_id);

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }
}

TEST_F(TestDataGraph, delete_edge) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Graph::Ptr graph = make_preset_graph();

  EXPECT_THROW_EQ(
    graph->delete_edge("node_5", "node_2"),
    std::out_of_range("Graph delete_edge failed, node [node_5] doesn't exist.")
  );

  EXPECT_THROW_EQ(
    graph->delete_edge("node_1", "node_5"),
    std::out_of_range("Graph delete_edge failed, node [node_5] doesn't exist.")
  );

  EXPECT_THROW_EQ(
    graph->delete_edge("node_1", "node_0"),
    std::out_of_range("Graph delete_edge failed, edge [node_1] to [node_0] doesn't exist.")
  );

  graph->delete_edge("node_1", "node_2");

  // Check edge is deleted
  {
    // node 1 updated
    std::string node_id = "node_1";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
    };

    Node::ConstPtr node = graph->get_node(node_id);

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }

  {
    // node 2 updated
    std::string node_id = "node_2";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_0", Edge("hard")},
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_3", Edge("hard")},
      {"node_4", Edge("soft")},
    };

    Node::ConstPtr node = graph->get_node(node_id);

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }
}

TEST_F(TestDataGraph, prune) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Graph::Ptr graph = make_preset_graph();
  Graph::Ptr graph_copy = make_preset_graph();

  // Add a few new empty nodes
  graph->add_node("node_5");
  graph->add_node("node_6");

  // prune
  graph->prune();

  // Back to the original graph.
  EXPECT_EQ(*graph, *graph_copy);
}

TEST_F(TestDataGraph, delete_node) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Graph::Ptr graph = make_preset_graph();

  EXPECT_THROW_EQ(
    graph->delete_node("node_5"),
    std::out_of_range("Graph delete_node failed, node [node_5] doesn't exist.")
  );

  graph->delete_node("node_1");

  // Node 1 disappeared
  EXPECT_FALSE(graph->has_node("node_1"));

  {
    // Node 2 updated
    std::string node_id = "node_2";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_0", Edge("hard")},
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_3", Edge("hard")},
      {"node_4", Edge("soft")},
    };

    Node::ConstPtr node = graph->get_node(node_id);

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }


  graph->delete_node("node_3");

  // Node 3 disappeared
  EXPECT_FALSE(graph->has_node("node_3"));

  {
    // Node 2 updated
    std::string node_id = "node_2";
    std::unordered_map<std::string, Edge> expected_inbound_edges {
      {"node_0", Edge("hard")},
    };
    std::unordered_map<std::string, Edge> expected_outbound_edges {
      {"node_4", Edge("soft")},
    };

    Node::ConstPtr node = graph->get_node(node_id);

    EXPECT_EQ(
      expected_inbound_edges,
      node->inbound_edges()
    );

    EXPECT_EQ(
      expected_outbound_edges,
      node->outbound_edges()
    );
  }
}

TEST_F(TestDataGraph, operators) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Graph::Ptr graph = make_preset_graph();
  Graph::Ptr graph_duplicate = make_preset_graph();

  // Extra node
  Graph::Ptr graph_extra_node = make_preset_graph();
  graph_extra_node->add_node("node_5");

  // different node id
  Graph::Ptr graph_different_node_id = make_preset_graph();
  graph_different_node_id->update_node("node_2", "node_5");

  // different edges
  Graph::Ptr graph_different_edges = make_preset_graph();
  graph_different_edges->delete_edge("node_0", "node_2");
  graph_different_edges->add_edge("node_0", "node_4");

  EXPECT_NE(graph, graph_duplicate);
  EXPECT_TRUE(*graph == *graph_duplicate);
  EXPECT_FALSE(*graph != *graph_duplicate);
  EXPECT_TRUE(*graph != Graph());
  EXPECT_TRUE(*graph != *graph_extra_node);
  EXPECT_TRUE(*graph != *graph_different_node_id);
  EXPECT_TRUE(*graph != *graph_different_edges);
}
