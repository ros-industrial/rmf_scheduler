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

#ifndef RMF_SCHEDULER__UTILS__SYSTEM_TIME_UTILS_HPP_
#define RMF_SCHEDULER__UTILS__SYSTEM_TIME_UTILS_HPP_

#include <chrono>
#include <string>

namespace rmf_scheduler
{
namespace utils
{

uint64_t to_ns(
  const std::chrono::system_clock::time_point & time_point);

std::chrono::system_clock::time_point to_chrono_time_point(
  uint64_t time_point);

uint64_t now();

uint64_t time_max();

char * to_localtime(
  uint64_t ns,
  const std::string & fmt = "%b %d %H:%M:%S %Y");

uint64_t from_localtime(
  const std::string & localtime,
  const std::string & fmt = "%b %d %H:%M:%S %Y");

char * get_default_timezone();

void set_timezone(const char * tz);

}  // namespace utils
}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__UTILS__SYSTEM_TIME_UTILS_HPP_
