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

#include "rmf2_scheduler/data/edge.hpp"

class TestDataEdge : public ::testing::Test
{
};

TEST_F(TestDataEdge, empty_create) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Edge edge;
  EXPECT_EQ(edge.type, "hard");
}

TEST_F(TestDataEdge, full_create) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Edge edge("soft");
  EXPECT_EQ(edge.type, "soft");
}

TEST_F(TestDataEdge, equal) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Edge edge("hard");

  {  // Equal
    Edge edge_to_compare("hard");
    EXPECT_EQ(edge, edge_to_compare);
  }

  {  // Different type
    Edge edge_to_compare("soft");
    EXPECT_NE(edge, edge_to_compare);
  }
}
