// Copyright 2025 ROS Industrial Consortium Asia Pacific
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

#include <string>

#include "gtest/gtest.h"

#include "rmf2_scheduler/data/timezone.hpp"

class TestTimeZone : public ::testing::Test
{
protected:
  void SetUp() override
  {
    using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
    unsetenv("TZ");
  }

  void TearDown() override
  {
  }
};

TEST_F(TestTimeZone, set_system_timezone) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  EXPECT_EQ(std::string(DEFAULT_SYSTEM_TIMEZONE), "UTC");
  EXPECT_EQ(std::string(get_timezone()), "UTC");

  set_system_timezone("UTC");
  EXPECT_EQ(std::string(get_timezone()), "UTC");

  set_system_timezone("Asia/Singapore");
  EXPECT_EQ(std::string(get_timezone()), "Asia/Singapore");
  EXPECT_EQ(std::string(tzname[0]), "+08");
}

TEST_F(TestTimeZone, scoped_timezone) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  EXPECT_EQ(std::string(get_timezone()), "UTC");

  {
    ScopedTimezone scoped_tz_sg("Asia/Singapore");

    EXPECT_EQ(std::string(get_timezone()), "Asia/Singapore");
    EXPECT_EQ(std::string(tzname[0]), "+08");

    {
      ScopedTimezone scoped_tz_sg("EST");

      EXPECT_EQ(std::string(get_timezone()), "EST");
      EXPECT_EQ(std::string(tzname[0]), "EST");
    }

    EXPECT_EQ(std::string(get_timezone()), "Asia/Singapore");
    EXPECT_EQ(std::string(tzname[0]), "+08");
  }

  EXPECT_EQ(std::string(get_timezone()), "UTC");
}
