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

#ifndef RMF_SCHEDULER__CONFLICT_RESOLVER__RESOLVER_HPP_
#define RMF_SCHEDULER__CONFLICT_RESOLVER__RESOLVER_HPP_

#include <cstdint>
#include <utility>
#include <vector>

#include "rmf_scheduler/conflict_resolver/identifier.hpp"

namespace rmf_scheduler
{
namespace conflict_resolver
{
class Resolver
{
public:
  /// Method to push new schedules into the cache
  /**
   * \param[in] from - start time of schedule in epoch format
   * \param[in] to - end time of schedule in epoch format
  */
  void push_intervals(uint64_t from, uint64_t to);

  /// Method to clear schedules from cache
  void clear_intervals();

  /// Method to resolve conflicts in cache
  /**
   * - Schedules are sorted based on start time
   * - Conflicted schedules are adjusted to fit the timelines
   *
   * \return resolved schedules
  */
  std::vector<std::pair<uint64_t, uint64_t>> resolve();

private:
  std::vector<std::pair<uint64_t, uint64_t>> events_;
};

}  // namespace conflict_resolver
}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__CONFLICT_RESOLVER__RESOLVER_HPP_
