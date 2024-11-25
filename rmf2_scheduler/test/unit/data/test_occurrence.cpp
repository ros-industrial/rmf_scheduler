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

#include "rmf2_scheduler/data/occurrence.hpp"

class TestDataOccurrence : public ::testing::Test
{
};

TEST_F(TestDataOccurrence, empty_create) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Occurrence occurrence;
  EXPECT_TRUE(occurrence.time == Time(0));
  EXPECT_EQ(occurrence.id, "");
}

TEST_F(TestDataOccurrence, full_create) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  {  // using Time
    Occurrence occurrence(
      Time(1672625700.0, 0),
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"
    );
    EXPECT_TRUE(occurrence.time == Time(1672625700.0, 0));
    EXPECT_EQ(occurrence.id, "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  }

  {  // using nanoseconds
    Occurrence occurrence(
      1672625700 * 1000LL * 1000LL * 1000LL,
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"
    );
    EXPECT_TRUE(occurrence.time == Time(1672625700.0, 0));
    EXPECT_EQ(occurrence.id, "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  }
}

TEST_F(TestDataOccurrence, equal) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Occurrence occurrence(
    Time(1672625700.0, 0),
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"
  );

  {  // Equal
    Occurrence occurrence_to_compare(
      Time(1672625700.0, 0),
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"
    );
    EXPECT_EQ(occurrence, occurrence_to_compare);
  }

  {  // Different ID
    Occurrence occurrence_to_compare(
      Time(1672625700.0, 0),
      "12345"
    );
    EXPECT_NE(occurrence, occurrence_to_compare);
  }

  {  // Different Time
    Occurrence occurrence_to_compare(
      Time(1672625760.0, 0),
      "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"
    );
    EXPECT_NE(occurrence, occurrence_to_compare);
  }
}
