// Copyright 2023-2024 ROS Industrial Consortium Asia Pacific
// Copyright 2017, 2018 Simon Rasmussen (refactor)
// Copyright 2015, 2016 Thomas Timm Andersen (original version)
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

#ifndef RMF2_SCHEDULER__LOG_HPP_
#define RMF2_SCHEDULER__LOG_HPP_

#include <memory>

#define LOG_DEBUG(...) \
  rmf2_scheduler::log::log(__FILE__, __LINE__, rmf2_scheduler::LogLevel::DEBUG, __VA_ARGS__)
#define LOG_WARN(...) \
  rmf2_scheduler::log::log(__FILE__, __LINE__, rmf2_scheduler::LogLevel::WARN, __VA_ARGS__)
#define LOG_INFO(...) \
  rmf2_scheduler::log::log(__FILE__, __LINE__, rmf2_scheduler::LogLevel::INFO, __VA_ARGS__)
#define LOG_ERROR(...) \
  rmf2_scheduler::log::log(__FILE__, __LINE__, rmf2_scheduler::LogLevel::ERROR, __VA_ARGS__)
#define LOG_FATAL(...) \
  rmf2_scheduler::log::log(__FILE__, __LINE__, rmf2_scheduler::LogLevel::FATAL, __VA_ARGS__)

namespace rmf2_scheduler
{

/// Different log levels
enum class LogLevel
{
  DEBUG = 0,
  INFO,
  WARN,
  ERROR,
  FATAL,
  NONE
};

namespace log
{

/// Inherit from this class to change the behavior when logging messages.
class LogHandler
{
public:
  virtual ~LogHandler() = default;
  /// Function to log a message
  /**
   * \param[in] file The log message comes from this file
   * \param[in] line The log message comes from this line
   * \param[in] loglevel Indicates the severity of the log message
   * \param[in] log Log message
   */
  virtual void log(const char * file, int line, LogLevel loglevel, const char * log) = 0;
};


/// Register a new LogHandler object, for handling log messages.
/**
 * \param[in] loghandler Pointer to the new object
 */
void registerLogHandler(std::unique_ptr<LogHandler> loghandler);

/// Unregister current log handler, this will enable default log handler.
void unregisterLogHandler();

/// Set log level this will disable messages with lower log level.
/**
 * \param[in] level desired log level
 */
void setLogLevel(LogLevel level);

/// get current log level.
/**
 * \returns log level
 */
LogLevel getLogLevel();

/// Log a message, this is used internally by the macros to unpack the log message.
/**
 * Use the macros instead of this function directly.
 *
 * \param[in] file The log message comes from this file
 * \param[in] line The log message comes from this line
 * \param[in] level Severity of the log message
 * \param[in] fmt Format string
 */
void log(const char * file, int line, LogLevel level, const char * fmt, ...);

}  // namespace log

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__LOG_HPP_
