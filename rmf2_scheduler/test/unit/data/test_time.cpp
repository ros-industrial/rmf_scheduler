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
// https://github.com/ros2/rclcpp/blob/master/rclcpp/test/rclcpp/test_time.cpp

#include <chrono>

#include "gtest/gtest.h"

#include "rmf2_scheduler/data/time.hpp"
#include "rmf2_scheduler/utils.hpp"
#include "../../utils/gtest_macros.hpp"

class TestTime : public ::testing::Test
{
};


static const int64_t HALF_SEC_IN_NS = 500 * 1000 * 1000;
static const int64_t ONE_SEC_IN_NS = 1000 * 1000 * 1000;
static const int64_t ONE_AND_HALF_SEC_IN_NS = 3 * HALF_SEC_IN_NS;

TEST_F(TestTime, operators) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Time old(1, 0);
  Time young(2, 0);

  EXPECT_TRUE(old < young);
  EXPECT_TRUE(young > old);
  EXPECT_TRUE(old <= young);
  EXPECT_TRUE(young >= old);
  EXPECT_FALSE(young == old);
  EXPECT_TRUE(young != old);

  Duration sub = young - old;
  EXPECT_EQ(sub.nanoseconds(), (young.nanoseconds() - old.nanoseconds()));
  EXPECT_EQ(sub, young - old);

  Time young_changed(young);
  young_changed -= Duration::from_nanoseconds(old.nanoseconds());
  EXPECT_EQ(sub.nanoseconds(), young_changed.nanoseconds());

  Time time = Time(0, 0);
  Time copy_constructor_time(time);
  Time assignment_op_time = Time(1, 0);
  assignment_op_time = time;

  EXPECT_TRUE(time == copy_constructor_time);
  EXPECT_TRUE(time == assignment_op_time);
}

TEST_F(TestTime, chrono_overloads) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  int64_t ns = 123456789l;
  std::chrono::system_clock::time_point t_system_clock{std::chrono::nanoseconds(ns)};
  std::chrono::steady_clock::time_point t_steady_clock{std::chrono::nanoseconds(ns)};
  auto t1 = Time(ns);
  auto t2 = Time(t_system_clock);
  auto t3 = Time(t_steady_clock);
  EXPECT_TRUE(t1 == t2);
  EXPECT_TRUE(t2 == t3);
  EXPECT_TRUE(t3 == t1);

  // to_chrono
  EXPECT_EQ(t2.to_chrono<std::chrono::system_clock>(), t_system_clock);
  EXPECT_EQ(t3.to_chrono<std::chrono::steady_clock>(), t_steady_clock);
}

TEST_F(TestTime, overflows) {
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Time max_time(std::numeric_limits<rs_time_point_value_t>::max());
  Duration one(1ns);
  Duration two(2ns);

  // Cross max
  EXPECT_THROW(max_time + one, std::overflow_error);
  EXPECT_THROW(Time(max_time) += one, std::overflow_error);
  EXPECT_NO_THROW(max_time - max_time);

  // Cross zero
  Time one_time(1);
  EXPECT_THROW(one_time - two, std::runtime_error);
  EXPECT_THROW(Time(one_time) -= two, std::runtime_error);

  Time two_time(2);
  EXPECT_NO_THROW(one_time - two_time);
}

TEST_F(TestTime, seconds) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  EXPECT_DOUBLE_EQ(0.0, Time(0, 0).seconds());
  EXPECT_DOUBLE_EQ(4.5, Time(4, 500000000).seconds());
  EXPECT_DOUBLE_EQ(2.5, Time(0, 2500000000).seconds());
}

TEST_F(TestTime, test_max) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
  const Time time_max = Time::max();
  const Time max_time(std::numeric_limits<int32_t>::max(), 999999999);
  EXPECT_DOUBLE_EQ(max_time.seconds(), time_max.seconds());
  EXPECT_EQ(max_time.nanoseconds(), time_max.nanoseconds());
}

TEST_F(TestTime, test_sum_operator) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)
  const Duration one(1ns);
  const Time test_time(0u);
  EXPECT_EQ(0u, test_time.nanoseconds());

  const Time new_time = one + test_time;
  EXPECT_EQ(1, new_time.nanoseconds());
}

TEST_F(TestTime, test_overflow_underflow_throws) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace std::chrono_literals;  // NOLINT(build/namespaces)

  Time test_time(0u);

  EXPECT_THROW_EQ(
    test_time = Time(INT64_MAX) + Duration(1ns),
    std::overflow_error("addition leads to int64_t overflow"));

  EXPECT_THROW_EQ(
    test_time = Time(INT64_MAX) - Duration(-1ns),
    std::overflow_error("time subtraction leads to int64_t overflow"));

  test_time = Time(INT64_MAX);
  EXPECT_THROW_EQ(
    test_time += Duration(1ns),
    std::overflow_error("addition leads to int64_t overflow"));

  test_time = Time(INT64_MAX);
  EXPECT_THROW_EQ(
    test_time -= Duration(-1ns),
    std::overflow_error("time subtraction leads to int64_t overflow"));

  EXPECT_THROW_EQ(
    test_time = Duration::from_nanoseconds(INT64_MAX) + Time(1),
    std::overflow_error("addition leads to int64_t overflow"));
}

TEST_F(TestTime, localtime)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  Time test_time(0l);
  rmf2_scheduler::utils::set_timezone("UTC");
  EXPECT_EQ(std::string(test_time.to_localtime()), "Jan 01 00:00:00 1970");
  EXPECT_TRUE(test_time == Time::from_localtime("Jan 01 00:00:00 1970"));

  test_time += Duration::from_seconds(31 * 24 * 60 * 60);
  EXPECT_EQ(
    std::string(test_time.to_localtime("%H:%M:%S %d/%m/%Y")),
    "00:00:00 01/02/1970"
  );
  EXPECT_EQ(
    test_time.nanoseconds(),
    Time::from_localtime(
      "00:00:00 01/02/1970", "%H:%M:%S %d/%m/%Y"
    ).nanoseconds()
  );
}
