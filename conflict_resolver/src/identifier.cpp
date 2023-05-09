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

#include <cmath>

#include <conflict_resolver/identifier.hpp>

namespace conflict_resolver
{

uint64_t Identifier::conflict_check(
  uint64_t from_start_time,
  uint64_t from_end_time,
  uint64_t with_start_time,
  uint64_t with_end_time)
{
  uint64_t duration_to_avoid_conflict = 0;

  if ((with_start_time >= from_start_time && with_start_time <= from_end_time) ||
    (with_end_time >= from_start_time && with_end_time <= from_end_time) ||
    (with_start_time <= from_start_time && with_end_time >= from_end_time))
  {
    // calculating duration for avoiding conflict
    duration_to_avoid_conflict = abs(with_start_time - from_end_time) + 1;
  }
  return duration_to_avoid_conflict;
}

}  // namespace conflict_resolver
