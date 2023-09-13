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

#include "rmf_scheduler_ros2/scheduler_log_handler.hpp"
#include "rclcpp/logging.hpp"

namespace rmf_scheduler_ros2
{

bool g_registered = false;
std::unique_ptr<SchedulerLogHandler> g_log_handler(new SchedulerLogHandler);

SchedulerLogHandler::SchedulerLogHandler() = default;

void SchedulerLogHandler::log(
  const char * file, int line, rmf_scheduler::LogLevel loglevel, const char * message)
{
  rcutils_log_location_t location = {"", file, static_cast<size_t>(line)};

  std::string logger_name = "RMF_Scheduler";
  if (!ns_.empty()) {
    logger_name = ns_ + '/' + logger_name;
  }
  switch (loglevel) {
    case rmf_scheduler::LogLevel::DEBUG:
      rcutils_log(
        &location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_DEBUG,
        logger_name.c_str(), "%s", message);
      break;
    case rmf_scheduler::LogLevel::INFO:
      rcutils_log(
        &location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_INFO,
        logger_name.c_str(), "%s", message);
      break;
    case rmf_scheduler::LogLevel::WARN:
      rcutils_log(
        &location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_WARN,
        logger_name.c_str(), "%s", message);
      break;
    case rmf_scheduler::LogLevel::ERROR:
      rcutils_log(
        &location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_ERROR,
        logger_name.c_str(), "%s", message);
      break;
    case rmf_scheduler::LogLevel::FATAL:
      rcutils_log(
        &location, RCUTILS_LOG_SEVERITY::RCUTILS_LOG_SEVERITY_FATAL,
        logger_name.c_str(), "%s", message);
      break;
    default:
      break;
  }
}

void register_scheduler_log_handler(const std::string & ns)
{
  if (g_registered == false) {
    g_log_handler->set_ns(ns);
    // Log level is decided by ROS2 log level
    rmf_scheduler::log::setLogLevel(rmf_scheduler::LogLevel::DEBUG);
    rmf_scheduler::log::registerLogHandler(std::move(g_log_handler));
    g_registered = true;
  }
}

void unregister_scheduler_log_handler()
{
  if (g_registered == true) {
    rmf_scheduler::log::unregisterLogHandler();
    g_registered = false;
  }
}

}  // namespace rmf_scheduler_ros2
