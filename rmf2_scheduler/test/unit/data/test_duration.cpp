// Copyright 2024 ROS Industrial Consortium Asia Pacific
// Copyright 2017 Open Source Robotics Foundation, Inc.
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
//
// Original file taken from:
// https://github.com/ros2/rclcpp/blob/master/rclcpp/test/rclcpp/test_duration.cpp

#include <chrono>
#include <limits>
#include <string>

#include "gtest/gtest.h"

#include "rmf2_scheduler/data/duration.hpp"
#include "../../utils/gtest_macros.hpp"


class TestDuration : public ::testing::Test
{
};

TEST_F(TestDuration, operators) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Duration old(1, 0);
  Duration young(2, 0);

  EXPECT_TRUE(old < young);
  EXPECT_TRUE(young > old);
  EXPECT_TRUE(old <= young);
  EXPECT_TRUE(young >= old);
  EXPECT_FALSE(young == old);
  EXPECT_TRUE(young != old);

  Duration add = old + young;
  EXPECT_EQ(add.nanoseconds(), old.nanoseconds() + young.nanoseconds());
  EXPECT_EQ(add, old + young);

  Duration sub = young - old;
  EXPECT_EQ(sub.nanoseconds(), young.nanoseconds() - old.nanoseconds());
  EXPECT_EQ(sub, young - old);

  Duration addequal = old;
  addequal += young;
  EXPECT_EQ(addequal.nanoseconds(), old.nanoseconds() + young.nanoseconds());
  EXPECT_EQ(addequal, old + young);

  Duration subequal = young;
  subequal -= old;
  EXPECT_EQ(subequal.nanoseconds(), young.nanoseconds() - old.nanoseconds());
  EXPECT_EQ(subequal, young - old);

  Duration scale = old * 3;
  EXPECT_EQ(scale.nanoseconds(), old.nanoseconds() * 3);

  Duration scaleequal = old;
  scaleequal *= 3;
  EXPECT_EQ(scaleequal.nanoseconds(), old.nanoseconds() * 3);

  Duration time = Duration(0, 0);
  Duration copy_constructor_duration(time);
  Duration assignment_op_duration = Duration(1, 0);
  (void)assignment_op_duration;
  assignment_op_duration = time;

  EXPECT_TRUE(time == copy_constructor_duration);
  EXPECT_TRUE(time == assignment_op_duration);
}

TEST_F(TestDuration, chrono_overloads) {
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  int64_t ns = 123456789l;
  auto chrono_ns = std::chrono::nanoseconds(ns);
  auto d1 = Duration::from_nanoseconds(ns);
  auto d2 = Duration(chrono_ns);
  auto d3 = Duration(123456789ns);
  EXPECT_EQ(d1, d2);
  EXPECT_EQ(d1, d3);
  EXPECT_EQ(d2, d3);

  // check non-nanosecond durations
  std::chrono::milliseconds chrono_ms(100);
  auto d4 = Duration(chrono_ms);
  EXPECT_EQ(chrono_ms, d4.to_chrono<std::chrono::nanoseconds>());
  std::chrono::duration<double, std::chrono::seconds::period> chrono_float_seconds(3.14);
  auto d5 = Duration(chrono_float_seconds);
  EXPECT_EQ(chrono_float_seconds, d5.to_chrono<decltype(chrono_float_seconds)>());
}

TEST_F(TestDuration, overflows) {
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  auto max = Duration::from_nanoseconds(std::numeric_limits<rs_duration_value_t>::max());
  auto min = Duration::from_nanoseconds(std::numeric_limits<rs_duration_value_t>::min());

  Duration one(1ns);
  Duration negative_one(-1ns);

  EXPECT_THROW(max + one, std::overflow_error);
  EXPECT_THROW(min - one, std::underflow_error);
  EXPECT_THROW(negative_one + min, std::underflow_error);
  // EXPECT_THROW(negative_one - max, std::underflow_error);

  Duration base_d = max * 0.3;
  EXPECT_THROW(base_d * 4, std::overflow_error);
  EXPECT_THROW(base_d * (-4), std::underflow_error);

  Duration base_d_neg = max * (-0.3);
  EXPECT_THROW(base_d_neg * (-4), std::overflow_error);
  EXPECT_THROW(base_d_neg * 4, std::underflow_error);
}

TEST_F(TestDuration, negative_duration) {
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Duration assignable_duration = Duration(0ns) - Duration(5, 0);

  // avoid windows converting a literal number less than -INT_MAX to unsigned int C4146
  int64_t expected_value = -5000;
  expected_value *= 1000 * 1000;
  EXPECT_EQ(expected_value, assignable_duration.nanoseconds());
}

TEST_F(TestDuration, maximum_duration) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Duration max_duration = Duration::max();
  Duration max(std::numeric_limits<int32_t>::max(), 999999999);

  EXPECT_EQ(max_duration, max);
}

static const int64_t HALF_SEC_IN_NS = 500 * 1000 * 1000;
static const int64_t ONE_SEC_IN_NS = 1000 * 1000 * 1000;
static const int64_t ONE_AND_HALF_SEC_IN_NS = 3 * HALF_SEC_IN_NS;
static const int64_t MAX_NANOSECONDS = std::numeric_limits<int64_t>::max();

TEST_F(TestDuration, from_seconds) {
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  EXPECT_EQ(Duration(0ns), Duration::from_seconds(0.0));
  EXPECT_EQ(Duration(0ns), Duration::from_seconds(0));
  EXPECT_EQ(Duration(1, HALF_SEC_IN_NS), Duration::from_seconds(1.5));
  EXPECT_EQ(
    Duration::from_nanoseconds(-ONE_AND_HALF_SEC_IN_NS),
    Duration::from_seconds(-1.5));
}

TEST_F(TestDuration, std_chrono_constructors) {
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  EXPECT_EQ(Duration(0ns), Duration(0.0s));
  EXPECT_EQ(Duration(0ns), Duration(0s));
  EXPECT_EQ(Duration(1, HALF_SEC_IN_NS), Duration(1.5s));
  EXPECT_EQ(Duration(-1, 0), Duration(-1s));
}

TEST_F(TestDuration, test_some_exceptions) {
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Duration test_duration(0ns);
  EXPECT_THROW_EQ(
    test_duration =
    Duration::from_nanoseconds(INT64_MAX) - Duration(-1ns),
    std::overflow_error("duration subtraction leads to int64_t overflow"));
  EXPECT_THROW_EQ(
    test_duration = test_duration * (std::numeric_limits<double>::infinity()),
    std::runtime_error("abnormal scale in Duration"));
}
