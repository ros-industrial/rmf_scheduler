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

#include <unordered_set>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "rmf2_scheduler/data/uuid.hpp"

TEST(TestDataUUID, GenUUIDTest) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using ::testing::MatchesRegex;

  std::unordered_set<std::string> uuids;
  for (int i = 0; i < 10; i++) {
    std::string new_uuid = gen_uuid();
    EXPECT_THAT(
      new_uuid,
      MatchesRegex("[a-fA-F0-9]{8}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{4}-[a-fA-F0-9]{12}"));
    EXPECT_TRUE(uuids.find(new_uuid) == uuids.end());
  }
}
