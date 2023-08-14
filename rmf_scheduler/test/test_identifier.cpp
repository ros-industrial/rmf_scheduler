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

#include <cstdint>
#include <chrono>

#include "gtest/gtest.h"

#include "rmf_scheduler/test_utils.hpp"
#include "rmf_scheduler/conflict_identifier.hpp"

TEST(TestIdentifier, SimpleConflictCheck)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)
  // No Conflict
  /**
   * For Example
   * Schedule A : 10 - 15
   * Schedule B : 25 - 30
  */
  EXPECT_FALSE(utils::simple_conflict_check(10, 15, 25, 30));

  // Conflict Type A
  /**
   * For Example
   * Schedule A : 10 - 15
   * Schedule B : 14 - 18
  */
  EXPECT_TRUE(utils::simple_conflict_check(10, 15, 14, 18));

  // Conflict Type B
  /**
   * For Example
   * Schedule A : 10 - 15
   * Schedule B : 7  - 11
  */
  EXPECT_TRUE(utils::simple_conflict_check(10, 15, 7, 11));

  // Conflict Type C
  /**
   * For Example
   * Schedule A : 10 - 15
   * Schedule B : 11 - 14
  */
  EXPECT_TRUE(utils::simple_conflict_check(10, 15, 11, 14));

  // Conflict Type D
  /**
   * For Example
   * Schedule A : 10 - 15
   * Schedule B : 5  - 21
  */
  EXPECT_TRUE(utils::simple_conflict_check(10, 15, 5, 21));
}

TEST(TestIdentifier, Categoriser)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  std::vector<Event> events =
    test_utils::load_clashing_events(5, 6, 10);

  auto events_by_filter = utils::categorise_by_filter(events, {"request::robot", "zone"});
  EXPECT_EQ(events_by_filter[0].size(), 5lu);

  EXPECT_EQ(events_by_filter[1].size(), 6lu);

  auto events_by_type = utils::categorise_by_type(events);
  EXPECT_EQ(events_by_type.size(), 2lu);
}

TEST(TestIdentifier, IdentifyConflict)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  std::vector<Event> events =
    test_utils::load_clashing_events(5, 6, 1000);

  auto time_point = std::chrono::steady_clock::now();
  auto conflicts1 = utils::identify_conflicts(events, {}, {"request::robot"});
  auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - time_point);
  time_point = std::chrono::steady_clock::now();
  auto conflicts2 = utils::identify_conflicts(events, {}, {"request::robot"}, "greedy");
  auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - time_point);

  std::cout << "Method - Optimal: " << duration1.count() << "ms" << std::endl;
  std::cout << "Method - Greedy: " << duration2.count() << "ms" << std::endl;
  std::cout << "Conflict1 size: " << conflicts1.size() << std::endl;
  // for (auto conflict : conflicts1) {
  //   std::cout << "- first: " << conflict.first
  //             << ", second: " << conflict.second
  //             << "\n  reason: filter: " << conflict.filter
  //             << ", detail: " << conflict.filtered_detail
  //             << std::endl;
  // }

  std::cout << "Conflict2 size: " << conflicts2.size() << std::endl;
  // for (auto conflict : conflicts2) {
  //   std::cout << "- first: " << conflict.first
  //             << ", second: " << conflict.second
  //             << "\n  reason: filter: " << conflict.filter
  //             << ", detail: " << conflict.filtered_detail
  //             << std::endl;
  // }
  EXPECT_EQ(conflicts1.size(), 4995lu);
  EXPECT_EQ(conflicts2.size(), 4995lu);

  conflicts1 = utils::identify_conflicts(events, {}, {"request::robot", "zone"});
  conflicts2 = utils::identify_conflicts(events, {}, {"request::robot", "zone"}, "greedy");
  std::cout << "Conflict1 size: " << conflicts1.size() << std::endl;
  std::cout << "Conflict1 size: " << conflicts2.size() << std::endl;
  EXPECT_EQ(conflicts1.size(), 15010lu);
  EXPECT_EQ(conflicts2.size(), 15010lu);
}
