// Copyright 2023 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF_SCHEDULER__SYSTEM_TIME_EXECUTOR_HPP_
#define RMF_SCHEDULER__SYSTEM_TIME_EXECUTOR_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rmf_scheduler/events_handler.hpp"
#include "rmf_scheduler/exception.hpp"

namespace rmf_scheduler
{

/// Executor that uses the system clock to schedule tasks.
/// Unless specificed otherwise, all methods in this class are thread-safe.
class SystemTimeExecutor
{
public:
  using Action = std::function<void ()>;

  SystemTimeExecutor();
  virtual ~SystemTimeExecutor();

  void add_action(const Event & event, Action action);

  void delete_action(const std::string & id);

  /// Spins the executor until `stop` is called.
  /*
   * Only one thread can spin the executor at once.
   */
  void spin();

  void stop();

protected:
  std::unordered_map<std::string, Action> actions_;
  std::vector<Action> pending_actions_;
  std::mutex mtx_;
  std::condition_variable cv_;
  std::atomic<bool> spinning_;

  EventsHandler eh_;

  void _tick();
};

class SystemTimeExecutorException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  SystemTimeExecutorException(const char * msg, Args && ... args)
  : ExceptionTemplate(msg, std::forward<Args>(args) ...)
  {
    add_prefix("SystemTimeExecutorException:\n  ");
  }
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__SYSTEM_TIME_EXECUTOR_HPP_
