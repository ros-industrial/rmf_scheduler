// Copyright 2023-2025 ROS Industrial Consortium Asia Pacific
// Copyright 2022 Open Source Robotics Foundation
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

#ifndef RMF2_SCHEDULER__SYSTEM_TIME_EXECUTOR_HPP_
#define RMF2_SCHEDULER__SYSTEM_TIME_EXECUTOR_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <map>
#include <utility>
#include <vector>

#include "rmf2_scheduler/data/time.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

struct SystemTimeAction
{
  using Work = std::function<void ()>;

  data::Time time;
  Work work;
};

/// Executor that uses the system clock to schedule tasks.
/// Unless specificed otherwise, all methods in this class are thread-safe.
class SystemTimeExecutor
{
public:
  RS_SMART_PTR_DEFINITIONS(SystemTimeExecutor)

  SystemTimeExecutor();
  virtual ~SystemTimeExecutor();

  std::string add_action(
    const SystemTimeAction & action
  );

  void delete_action(
    const std::string & id
  );

  /// Spins the executor until `stop` is called.
  /*
   * Only one thread can spin the executor at once.
   */
  void spin();

  void stop();

private:
  RS_DISABLE_COPY(SystemTimeExecutor)

  using SystemTimeActionIterator = std::unordered_map<std::string, SystemTimeAction>::iterator;

  std::vector<SystemTimeActionIterator> _lookup_earliest_actions();

  void _delete_time_lookup(const std::string & id, const data::Time & time);

  void _tick();

  std::unordered_map<std::string, SystemTimeAction> actions_;
  std::vector<SystemTimeAction> pending_actions_;
  std::multimap<rs_time_point_value_t, std::string> time_lookup_;

  std::mutex mtx_;
  std::condition_variable cv_;
  std::atomic<bool> spinning_;
};

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__SYSTEM_TIME_EXECUTOR_HPP_
