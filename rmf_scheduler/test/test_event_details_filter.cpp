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
#include "rmf_scheduler/parser.hpp"

TEST(TestEventDetailFilter, Basic) {
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)
  std::string event_details = R"({"test1": {"test2": {"test3": "hello_world"}}})";
  std::string filtered_detail;
  bool result = parser::filter_event_details<std::string>(
    event_details, "test1::test2::test3", filtered_detail);
  EXPECT_TRUE(result);
  EXPECT_EQ(filtered_detail, "hello_world");

  std::vector<bool> filtered_results;
  std::vector<nlohmann::json> filtered_details;

  parser::batch_filter_event_details(
    event_details, {"test1", "random", "test2", "test1::test2::test4"},
    filtered_results, filtered_details);

  EXPECT_TRUE(filtered_results[0]);
  EXPECT_EQ(filtered_details[0].dump(), R"({"test2":{"test3":"hello_world"}})");
  EXPECT_FALSE(filtered_results[1]);
  EXPECT_FALSE(filtered_results[2]);
  EXPECT_FALSE(filtered_results[3]);
}
