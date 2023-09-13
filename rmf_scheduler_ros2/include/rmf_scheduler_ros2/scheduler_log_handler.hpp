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

#ifndef RMF_SCHEDULER_ROS2__SCHEDULER_LOG_HANDLER_HPP_
#define RMF_SCHEDULER_ROS2__SCHEDULER_LOG_HANDLER_HPP_

#include <string>

#include "rmf_scheduler/log.hpp"

namespace rmf_scheduler_ros2
{

void register_scheduler_log_handler(const std::string & ns = "");


void unregister_scheduler_log_handler();

class SchedulerLogHandler : public rmf_scheduler::log::LogHandler
{
public:
  SchedulerLogHandler();

  void log(
    const char * file, int line, rmf_scheduler::LogLevel loglevel,
    const char * message) override;

  const std::string & ns() const
  {
    return ns_;
  }

private:
  std::string ns_;

  void set_ns(const std::string & ns)
  {
    ns_ = ns;
  }

  friend void register_scheduler_log_handler(const std::string & ns);
};

}  // namespace rmf_scheduler_ros2

#endif  // RMF_SCHEDULER_ROS2__SCHEDULER_LOG_HANDLER_HPP_
