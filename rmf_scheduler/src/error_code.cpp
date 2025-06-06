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

#include "nlohmann/json.hpp"
#include "rmf_scheduler/error_code.hpp"
#include "rmf_scheduler/exception.hpp"
#include "rmf_scheduler/log.hpp"

namespace rmf_scheduler
{
std::string ErrorCode::json() const
{
  return nlohmann::json {
    {"value", val},
    {"detail", detail}
  }.dump();
}

ErrorCode ErrorCode::resolve_error_code(std::function<void()> && func)
{
  try {
    func();
  } catch (const exception::ExceptionTemplate & e) {
    // Internal error
    RS_LOG_ERROR(e.what());
    return {
      e.code(),
      e.what()
    };
  } catch (const std::exception & e) {
    // Other errors
    RS_LOG_ERROR(e.what());
    return {
      FAILURE,
      e.what()
    };
  }
  return SUCCESS;
}

}  // namespace rmf_scheduler
