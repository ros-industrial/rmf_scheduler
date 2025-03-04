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

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

#include "rmf_scheduler/utils/system_time_utils.hpp"

namespace rmf_scheduler
{
namespace utils
{

uint64_t to_ns(
  const std::chrono::system_clock::time_point & time_point)
{
  auto chrono_duration = time_point.time_since_epoch();
  uint64_t duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
    chrono_duration).count();
  return duration;
}

std::chrono::system_clock::time_point to_chrono_time_point(
  uint64_t time_point)
{
  return std::chrono::system_clock::time_point(std::chrono::nanoseconds(time_point));
}

uint64_t now()
{
  return to_ns(std::chrono::system_clock::now());
}

uint64_t time_max()
{
  return to_ns(std::chrono::system_clock::time_point::max());
}

char * to_localtime(
  uint64_t ns,
  const std::string & fmt)
{
  time_t t = ns / 1e9;
  std::tm ltime;
  localtime_r(&t, &ltime);
  static char mbstr[50];
  std::strftime(mbstr, sizeof(mbstr), fmt.c_str(), &ltime);
  return mbstr;
}

uint64_t from_localtime(
  const std::string & localtime,
  const std::string & fmt)
{
  std::tm t = {};
  std::istringstream ss(localtime);
  ss >> std::get_time(&t, fmt.c_str());
  if (ss.fail()) {
    return 0;
  }
  time_t t_s = std::mktime(&t);
  return static_cast<uint64_t>(t_s) * 1e9;
}

void set_timezone(const char * tz)
{
#if defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
  setenv("TZ", tz, 1);
  tzset();
#endif
}


}  // namespace utils
}  // namespace rmf_scheduler
