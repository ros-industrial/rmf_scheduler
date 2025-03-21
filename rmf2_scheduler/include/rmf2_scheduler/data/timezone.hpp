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

#ifndef RMF2_SCHEDULER__DATA__TIMEZONE_HPP_
#define RMF2_SCHEDULER__DATA__TIMEZONE_HPP_

#include <cstdlib>
#include <ctime>

namespace rmf2_scheduler
{

namespace data
{

static constexpr char DEFAULT_SYSTEM_TIMEZONE[] = "UTC";

inline void set_system_timezone(const char * tz)
{
  setenv("TZ", tz, 1);
  tzset();
}

inline const char * get_timezone()
{
  const char * timezone = getenv("TZ");
  return timezone ? timezone : DEFAULT_SYSTEM_TIMEZONE;
}

class ScopedTimezone
{
public:
  explicit ScopedTimezone(const char * tz)
  {
    // Retrieve system timezone
    system_timezone_ = get_timezone();
    setenv("TZ", tz, 1);
    tzset();
  }

  virtual ~ScopedTimezone()
  {
    setenv("TZ", system_timezone_, 1);
    tzset();
  }

private:
  const char * system_timezone_;
};

}  // namespace data

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__DATA__TIMEZONE_HPP_
