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

#include "croncpp/croncpp.h"

#include "rmf_scheduler/window.hpp"
#include "rmf_scheduler/utils/system_time_utils.hpp"

namespace rmf_scheduler
{

WindowUtils::WindowUtils(
  const std::string & cron,
  const std::string & timezone)
{
  // Create cron to validate
  // existing occurrence if it has more than 1 occurrence
  try {
    cron_ = std::make_unique<cron::cronexpr>(cron::make_cron(cron));
  } catch (const cron::bad_cronexpr & e) {
    throw exception::WindowInvalidCronException(
            "Cannot create window. "
            "Invalid cron: %s", e.what());
  }
  tz_ = timezone;
}

WindowUtils::~WindowUtils()
{
}

Window WindowUtils::get_window(uint64_t time)
{
  Window window;
  window.start_time = time;
  time_t _time = time / 1e9;
  utils::set_timezone(tz_.c_str());
  time_t next_time = cron::cron_next(*cron_, _time);
  window.end_time = static_cast<uint64_t>(next_time * 1e9);
  return window;
}

}  // namespace rmf_scheduler
