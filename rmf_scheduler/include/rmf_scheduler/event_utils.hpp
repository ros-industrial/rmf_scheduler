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

#ifndef RMF_SCHEDULER__EVENT_UTILS_HPP_
#define RMF_SCHEDULER__EVENT_UTILS_HPP_

#include "rmf_scheduler/event.hpp"

namespace rmf_scheduler
{
namespace utils
{

bool is_equal(const Event & lhs, const Event & rhs)
{
  return
    lhs.description == rhs.description &&
    lhs.type == rhs.type &&
    lhs.start_time == rhs.start_time &&
    lhs.duration == rhs.duration &&
    lhs.id == rhs.id &&
    lhs.series_id == rhs.series_id &&
    lhs.dag_id == rhs.dag_id;
}

}  // namespace utils
}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__EVENT_UTILS_HPP_
