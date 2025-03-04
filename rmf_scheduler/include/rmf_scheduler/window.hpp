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

#ifndef RMF_SCHEDULER__WINDOW_HPP_
#define RMF_SCHEDULER__WINDOW_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rmf_scheduler/exception.hpp"
#include "rmf_scheduler/error_code.hpp"

namespace cron
{

class cronexpr;

}  // namespace cron

namespace rmf_scheduler
{

struct Window
{
  uint64_t start_time;
  uint64_t end_time;
};

class WindowUtils
{
public:
  explicit WindowUtils(
    const std::string & cron,
    const std::string & timezone);

  virtual ~WindowUtils();

  Window get_window(uint64_t time);

private:
  std::unique_ptr<cron::cronexpr> cron_;
  std::string tz_;
};

namespace exception
{

class WindowInvalidCronException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  WindowInvalidCronException(const char * msg, Args && ... args)
  : ExceptionTemplate(
      ErrorCode::FAILURE,
      msg, std::forward<Args>(args) ...)
  {
    add_prefix("WindowInvalidCronException:\n  ");
  }
};
}  // namespace exception

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__WINDOW_HPP_
