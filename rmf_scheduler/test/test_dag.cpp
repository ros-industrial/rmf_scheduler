// Copyright 2023 ROS Industrial Consortium Asia Pacific
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
#include "rmf_scheduler/data/dag.hpp"
#include "rmf_scheduler/test_utils.hpp"
#include "rmf_scheduler/sanitizer_macro.hpp"
#include "graphviz/gvc.h"

class TestDAG : public ::testing::Test
{
protected:
  void SetUp() override
  {
    using namespace rmf_scheduler::data;  // NOLINT(build/namespaces)
    dag_description_ = {
      {"task1", {}},
      {"task2", {"task4", "task3"}},
      {"task3", {"task1"}},
      {"task4", {"task1"}}
    };
    dag_ = DAG(dag_description_);
  }

  void TearDown() override
  {
  }

  rmf_scheduler::data::DAG dag_;
  rmf_scheduler::data::DAG::Description dag_description_;
};

TEST_F(TestDAG, BasicDAGCRUD) {
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Validate dependency information
  auto out = dag_.description();
  for (auto & itr : out) {
    // std::cout << itr.first << ": ";
    // for (auto & dep_itr : itr.second) {
    //   std::cout << dep_itr << ' ';
    // }
    // std::cout << std::endl;
    EXPECT_TRUE(
      test_utils::is_vector_equal(
        itr.second, dag_description_.at(itr.first)
    ));
  }

  // Validate entry and ending node
  EXPECT_EQ(dag_.entry_nodes(), std::unordered_set<std::string>{"task1"});

  EXPECT_EQ(dag_.end_nodes(), std::unordered_set<std::string>{"task2"});

  // Add a new node
  dag_.add_node("task5");
  dag_.add_node("task6");
  dag_.add_dependency("task5", {"task6", "task2"});

  // Validate additional dependency information
  EXPECT_TRUE(
    test_utils::is_vector_equal(
      dag_.get_dependency_info("task5"),
      {"task6", "task2"}
  ));
  EXPECT_TRUE(dag_.get_dependency_info("task6").empty());

  // Add dependencies that exists (should ignore)
  dag_.add_dependency("task5", {"task6", "task2"});
  // Dependency info unchanged
  EXPECT_TRUE(
    test_utils::is_vector_equal(
      dag_.get_dependency_info("task5"),
      {"task6", "task2"}
  ));

  // out = dag_.description();
  // for (auto & itr : out) {
  //   std::cout << itr.first << ": ";
  //   for (auto & dep_itr : itr.second) {
  //     std::cout << dep_itr << ' ';
  //   }
  //   std::cout << std::endl;
  // }

  // Check clear dependency function
  EXPECT_FALSE(dag_.get_dependency_info("task2").empty());
  dag_.clear_dependency("task2");
  EXPECT_TRUE(dag_.get_dependency_info("task2").empty());
  EXPECT_TRUE(
    test_utils::is_vector_equal(
      dag_.get_dependency_info("task5"),
      {"task6", "task2"}
  ));

  // Check delete node function
  dag_.delete_node("task1");
  EXPECT_FALSE(dag_.has_node("task1"));
  out = dag_.description();
  for (auto & itr : out) {
    // std::cout << itr.first << ": ";
    for (auto & dep_itr : itr.second) {
      // std::cout << dep_itr << ' ';
      EXPECT_NE(dep_itr, "task1");
    }
  }
}

TEST_F(TestDAG, DAGBTXML) {
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  std::string bt_id{"5"};
  std::unordered_map<std::string, std::string> map = {
    {"task1", "TrayID: A0221, Vehicle: Robot2, OriginID: 35, DestinationID: 6"},
    {"task2", "TrayID: A0221, Vehicle: Robot2, OriginID: 40, DestinationID: 8"},
    {"task3", "TrayID: A0221, Vehicle: Robot2, OriginID: 25, DestinationID: 90"},
    {"task4", "TrayID: A0221, Vehicle: Robot2, OriginID: 10, DestinationID: 15"},
    {"task5", "TrayID: A0221, Vehicle: Robot2, OriginID: 15, DestinationID: 13"},
    {"task6", "TrayID: A0221, Vehicle: Robot2, OriginID: 13, DestinationID: 28"},
    {"task7", "TrayID: A0221, Vehicle: Robot2, OriginID: 19, DestinationID: 35"},
    {"task8", "TrayID: A0221, Vehicle: Robot2, OriginID: 60, DestinationID: 65"},
    {"task9", "TrayID: A0221, Vehicle: Robot2, OriginID: 49, DestinationID: 40"},
    {"task10", "TrayID: A0221, Vehicle: Robot2, OriginID: 20, DestinationID: 29"},
    {"task11", "TrayID: A0221, Vehicle: Robot2, OriginID: 80, DestinationID: 35"},
    {"task12", "TrayID: A0221, Vehicle: Robot2, OriginID: 65, DestinationID: 54"},
    {"task13", "TrayID: A0221, Vehicle: Robot2, OriginID: 30, DestinationID: 32"},
    {"task14", "TrayID: A0221, Vehicle: Robot2, OriginID: 29, DestinationID: 4"}
  };
  static const char * xml_text =
    R"(<root BTCPP_format="4">
    <BehaviorTree ID="5">
        <Sequence>
            <Script code="MO1:=&apos;A0221,task1,35,6&apos;"/>
            <MovementOrder MO="{MO1}"/>
            <Parallel>
                <Script code="MO2:=&apos;A0221,task3,25,90&apos;"/>
                <MovementOrder MO="{MO2}"/>
                <Script code="MO3:=&apos;A0221,task4,10,15&apos;"/>
                <MovementOrder MO="{MO3}"/>
            </Parallel>
            <Script code="MO4:=&apos;A0221,task2,40,8&apos;"/>
            <MovementOrder MO="{MO4}"/>
        </Sequence>
    </BehaviorTree>
</root>
)";
  std::function<std::string(const std::string &)> f5 = [&map](const std::string & id)
    -> std::string {
      std::string description;
      description = map.at(id);
      return description;
    };
  auto str = dag_.generate_bt_xml(bt_id, f5);
  EXPECT_EQ(str, xml_text);
  // std::cout << str << std::endl;
}
// Graphviz leaks due to its upstream dependencies
// https://gitlab.com/graphviz/graphviz/-/issues/1461
// Disable address sanitizer
#ifndef __SANITIZE_ADDRESS__
#ifndef __SANITIZE_THREAD__
TEST_F(TestDAG, DAGDOT) {
  auto dot = dag_.dot();
  GVC_t * gvc = gvContext();
  Agraph_t * g = agmemread(dot.c_str());

  gvLayout(gvc, g, "dot");
  gvRender(gvc, g, "plain", stdout);
  gvFreeLayout(gvc, g);
  agclose(g);
  gvFreeContext(gvc);
}
#endif
#endif

TEST_F(TestDAG, DAGCheckCyclic) {
  // Original graph is not cyclic
  EXPECT_FALSE(dag_.is_cyclic());

  // Loop task1 -> task3 -> task1
  dag_.add_dependency("task1", {"task3"});
  EXPECT_TRUE(dag_.is_cyclic());

  // Back to not acyclic
  dag_.clear_dependency("task1");
  EXPECT_FALSE(dag_.is_cyclic());

  // Loop task1 -> task3 -> task2 -> task 1
  //  and task1 -> task4 -> task2 -> task 1
  dag_.add_dependency("task1", {"task2"});
  EXPECT_TRUE(dag_.is_cyclic());

  bool exception_caught = false;
  try {
    dag_.add_dependency("task1", {"task3"}, true);
  } catch (rmf_scheduler::exception::DAGCyclicException & e) {
    std::cerr << "exception caught:\n" << e.what() << std::endl;
    exception_caught = true;
  }
  EXPECT_TRUE(exception_caught);

  exception_caught = false;
  try {
    dag_.clear_dependency("task1");
    dag_.add_dependency("task1", {"task2"}, true);
  } catch (rmf_scheduler::exception::DAGCyclicException & e) {
    std::cerr << "exception caught:\n" << e.what() << std::endl;
    exception_caught = true;
  }
  EXPECT_TRUE(exception_caught);
}
