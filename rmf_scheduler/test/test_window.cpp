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

#include "rmf_scheduler/window.hpp"
#include "rmf_scheduler/utils/system_time_utils.hpp"

TEST(TestWindow, ValidWindow)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)
  WindowUtils window_utils("0 0 0 */1 * *", "Asia/Singapore");

  utils::set_timezone("Asia/Singapore");
  Window window = window_utils.get_window(utils::from_localtime("Oct 3 10:15:00 2023"));

  std::cout << "Window start time: " << utils::to_localtime(window.start_time)
            << "Window end time: " << utils::to_localtime(window.end_time) << std::endl;

  EXPECT_EQ(window.start_time, utils::from_localtime("Oct 3 10:15:00 2023"));
  EXPECT_EQ(window.end_time, utils::from_localtime("Oct 4 00:00:00 2023"));
}
