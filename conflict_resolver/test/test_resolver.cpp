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

#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include <conflict_resolver/resolver.hpp>

class TestResolver : public ::testing::Test
{
protected:
  void SetUp() override
  {
  }

  void TearDown() override
  {
    resolver_.clear_intervals();
  }

  conflict_resolver::Resolver resolver_;
};

/// Resolver with no conflict
/**
 * For Example
 * Schedule A : 5  - 10
 * Schedule B : 11 - 15
 * Schedule C : 20 - 25
 * Schedule D : 27 - 32
*/
TEST_F(TestResolver, ResolveWithNoConflicts)
{
  resolver_.push_intervals(20, 25);
  resolver_.push_intervals(5, 10);
  resolver_.push_intervals(27, 32);
  resolver_.push_intervals(11, 15);

  std::vector<std::pair<uint64_t, uint64_t>> result = resolver_.resolve();
  EXPECT_EQ(result.size(), std::make_unsigned_t<int64_t>(4));
  EXPECT_EQ(result[0].first, std::make_unsigned_t<int64_t>(5));
  EXPECT_EQ(result[0].second, std::make_unsigned_t<int64_t>(10));
  EXPECT_EQ(result[1].first, std::make_unsigned_t<int64_t>(11));
  EXPECT_EQ(result[1].second, std::make_unsigned_t<int64_t>(15));
  EXPECT_EQ(result[2].first, std::make_unsigned_t<int64_t>(20));
  EXPECT_EQ(result[2].second, std::make_unsigned_t<int64_t>(25));
  EXPECT_EQ(result[3].first, std::make_unsigned_t<int64_t>(27));
  EXPECT_EQ(result[3].second, std::make_unsigned_t<int64_t>(32));
}

/// Resolver with only two conflicts happening at the same time
/**
 * For Example
 * Schedule A :  11 - 15
 * Schedule B :  14 - 20
 * Schedule C :  25 - 30
 * Schedule D :  29 - 34
*/
TEST_F(TestResolver, ResolveWithConflictsFromMultipleTypesOnDifferentTimeline)
{
  resolver_.push_intervals(25, 30);
  resolver_.push_intervals(29, 34);
  resolver_.push_intervals(11, 15);
  resolver_.push_intervals(14, 20);

  std::vector<std::pair<uint64_t, uint64_t>> result = resolver_.resolve();
  EXPECT_EQ(result.size(), std::make_unsigned_t<int64_t>(4));
  EXPECT_EQ(result[0].first, std::make_unsigned_t<int64_t>(11));
  EXPECT_EQ(result[0].second, std::make_unsigned_t<int64_t>(15));
  EXPECT_EQ(result[1].first, std::make_unsigned_t<int64_t>(16));
  EXPECT_EQ(result[1].second, std::make_unsigned_t<int64_t>(22));
  EXPECT_EQ(result[2].first, std::make_unsigned_t<int64_t>(25));
  EXPECT_EQ(result[2].second, std::make_unsigned_t<int64_t>(30));
  EXPECT_EQ(result[3].first, std::make_unsigned_t<int64_t>(31));
  EXPECT_EQ(result[3].second, std::make_unsigned_t<int64_t>(36));
}

/// Resolver with multiple conflicts happening at the same time
/**
 * For Example
 * Schedule A :  5  - 21
 * Schedule B :  7  - 11
 * Schedule C :  10 - 15
 * Schedule D :  11 - 14
 * Schedule E :  14 - 18
 * Schedule F :  25 - 30
*/
TEST_F(TestResolver, ResolveWithConflictsFromMultipleTypesOnSameTimeline)
{
  resolver_.push_intervals(10, 15);
  resolver_.push_intervals(25, 30);
  resolver_.push_intervals(14, 18);
  resolver_.push_intervals(7, 11);
  resolver_.push_intervals(11, 14);
  resolver_.push_intervals(5, 21);

  std::vector<std::pair<uint64_t, uint64_t>> result = resolver_.resolve();
  EXPECT_EQ(result.size(), std::make_unsigned_t<int64_t>(6));
  EXPECT_EQ(result[0].first, std::make_unsigned_t<int64_t>(5));
  EXPECT_EQ(result[0].second, std::make_unsigned_t<int64_t>(21));
  EXPECT_EQ(result[1].first, std::make_unsigned_t<int64_t>(22));
  EXPECT_EQ(result[1].second, std::make_unsigned_t<int64_t>(26));
  EXPECT_EQ(result[2].first, std::make_unsigned_t<int64_t>(27));
  EXPECT_EQ(result[2].second, std::make_unsigned_t<int64_t>(32));
  EXPECT_EQ(result[3].first, std::make_unsigned_t<int64_t>(33));
  EXPECT_EQ(result[3].second, std::make_unsigned_t<int64_t>(36));
  EXPECT_EQ(result[4].first, std::make_unsigned_t<int64_t>(37));
  EXPECT_EQ(result[4].second, std::make_unsigned_t<int64_t>(41));
  EXPECT_EQ(result[5].first, std::make_unsigned_t<int64_t>(42));
  EXPECT_EQ(result[5].second, std::make_unsigned_t<int64_t>(47));
}
