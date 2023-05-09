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

#include <cstdint>

#include <conflict_resolver/identifier.hpp>

/// No Conflict
/**
 * For Example
 * Schedule A : 10 - 15
 * Schedule B : 25 - 30
*/
TEST(TestIdentifier, NoConflict)
{
  using Identifier = conflict_resolver::Identifier;

  Identifier identifier;
  uint64_t result = 0;

  uint64_t duration = identifier.conflict_check(10, 15, 25, 30);
  EXPECT_EQ(duration, result);
}

/// Conflict Type A
/**
 * For Example
 * Schedule A : 10 - 15
 * Schedule B : 14 - 18
*/
TEST(TestIdentifier, ConflictTypeA)
{
  using Identifier = conflict_resolver::Identifier;

  Identifier identifier;
  uint64_t result = 2;

  uint64_t duration = identifier.conflict_check(10, 15, 14, 18);
  EXPECT_EQ(duration, result);
}

/// Conflict Type B
/**
 * For Example
 * Schedule A : 10 - 15
 * Schedule B : 7  - 11
*/
TEST(TestIdentifier, ConflictTypeB)
{
  using Identifier = conflict_resolver::Identifier;

  Identifier identifier;
  uint64_t result = 9;

  uint64_t duration = identifier.conflict_check(10, 15, 7, 11);
  EXPECT_EQ(duration, result);
}

/// Conflict Type C
/**
 * For Example
 * Schedule A : 10 - 15
 * Schedule B : 11 - 14
*/
TEST(TestIdentifier, ConflictTypeC)
{
  using Identifier = conflict_resolver::Identifier;

  Identifier identifier;
  uint64_t result = 5;

  uint64_t duration = identifier.conflict_check(10, 15, 11, 14);
  EXPECT_EQ(duration, result);
}

/// Conflict Type D
/**
 * For Example
 * Schedule A : 10 - 15
 * Schedule B : 5  - 21
*/
TEST(TestIdentifier, ConflictTypeD)
{
  using Identifier = conflict_resolver::Identifier;

  Identifier identifier;
  uint64_t result = 11;

  uint64_t duration = identifier.conflict_check(10, 15, 5, 21);
  EXPECT_EQ(duration, result);
}
