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

#include <cstdio>
#include "rmf_scheduler/default_log_handler.hpp"

namespace rmf_scheduler
{

namespace log
{

DefaultLogHandler::DefaultLogHandler() = default;

void DefaultLogHandler::log(const char * file, int line, LogLevel loglevel, const char * log)
{
  switch (loglevel) {
    case LogLevel::INFO:
      printf("%s%s %i: %s \n", "INFO ", file, line, log);
      break;
    case LogLevel::DEBUG:
      printf("%s%s %i: %s \n", "DEBUG ", file, line, log);
      break;
    case LogLevel::WARN:
      printf("%s%s %i: %s \n", "WARN ", file, line, log);
      break;
    case LogLevel::ERROR:
      printf("%s%s %i: %s \n", "ERROR ", file, line, log);
      break;
    case LogLevel::FATAL:
      printf("%s%s %i: %s \n", "FATAL ", file, line, log);
      break;
    default:
      break;
  }
}

}  // namespace log

}  // namespace rmf_scheduler
