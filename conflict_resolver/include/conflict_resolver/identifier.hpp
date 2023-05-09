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

#ifndef CONFLICT_RESOLVER__IDENTIFIER_HPP_
#define CONFLICT_RESOLVER__IDENTIFIER_HPP_

#include <cstdint>

namespace conflict_resolver
{
class Identifier
{
public:
  /// Method to check conflicts in given schedules
  /**
   * Example Conflicts:
   * - schedule a: 10 - 15, schedule b:  5 - 11
   * - schedule a: 10 - 15, schedule b: 14 - 19
   * - schedule a: 10 - 15, schedule b: 12 - 14
   * - schedule a: 10 - 15, schedule b:  8 - 17
   *
   * \param[in] from_start_time - Schedule A start time
   * \param[in] from_end_time - Schedule A end time
   * \param[in] with_start_time - Schedule B start time
   * \param[in] with_end_time - Schedule B end time
   * \return[out] Duration of conflict between schedules
   */
  uint64_t conflict_check(
    uint64_t from_start_time,
    uint64_t from_end_time,
    uint64_t with_start_time,
    uint64_t with_end_time);
};

}  // namespace conflict_resolver

#endif  // CONFLICT_RESOLVER__IDENTIFIER_HPP_
