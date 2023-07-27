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

#ifndef RMF_SCHEDULER__ERROR_CODE_HPP_
#define RMF_SCHEDULER__ERROR_CODE_HPP_

#include <cstdint>
#include <string>
#include <functional>

#define RMF_SCHEDULER_RESOLVE_ERROR_CODE(f) \
  rmf_scheduler::ErrorCode::resolve_error_code([ = ]() {f;})

namespace rmf_scheduler
{

/// Error code for RMF scheduler
struct ErrorCode
{
public:
  static constexpr int32_t OVERALL = 0xff;
  static constexpr int32_t SUCCESS = 0x00;
  static constexpr int32_t FAILURE = 0x01;

  // Error type
  static constexpr int32_t ERROR_TYPE = 0xff << 8;
  static constexpr int32_t INVALID_ID = 0x01 << 8;
  static constexpr int32_t INVALID_LOGIC = 0x02 << 8;
  static constexpr int32_t INVALID_SCHEMA = 0x03 << 8;
  static constexpr int32_t MULTIPLE_ACCESS = 0x04 << 8;

  // Error data type
  static constexpr int32_t FIELD = 0xff << 16;
  static constexpr int32_t NO_FIELD = 0xfe << 16;
  static constexpr int32_t INVALID_EVENT = 0x01 << 16;
  static constexpr int32_t INVALID_DEPENDENCY = 0x02 << 16;
  static constexpr int32_t INVALID_SERIES = 0x03 << 16;

  ErrorCode(
    int code = 0,
    const std::string & msg = "")  // NOLINT(runtime/explicit)
  : val(code),
    detail(msg)
  {
  }

  explicit operator bool() const
  {
    return get(OVERALL) == SUCCESS;
  }

  int get(int mask) const
  {
    return val & mask;
  }

  std::string str(const std::string & delimiter = ",") const
  {
    if (*this) {
      return "SUCCESS";
    }

    // Add ERROR TYPE
    std::string out = "FAILURE" + delimiter;
    switch (get(ERROR_TYPE)) {
      case INVALID_ID:
        out += "INVALID_ID";
        break;
      case INVALID_LOGIC:
        out += "INVALID_LOGIC";
        break;
      case INVALID_SCHEMA:
        out += "INVALID_SCHEMA";
        break;
      case MULTIPLE_ACCESS:
        out += "MULTIPLE_ACCESS";
        break;
      default:
        out += "UNKNOWN_ERROR_TYPE";
        break;
    }

    // Add INVALID FIELD
    out += delimiter;
    switch (get(FIELD)) {
      case INVALID_EVENT:
        out += "INVALID_EVENT";
        break;
      case INVALID_DEPENDENCY:
        out += "INVALID_DEPENDENCY";
        break;
      case INVALID_SERIES:
        out += "INVALID_SERIES";
        break;
      case NO_FIELD:
        break;
      default:
        out += "UNKNOWN_FIELD";
        break;
    }

    // Add additional details at the back
    if (!detail.empty()) {
      out += "\n";
      out += detail;
    }

    return out;
  }

  std::string json() const;

  int32_t val;
  std::string detail;

  static ErrorCode resolve_error_code(std::function<void()> && func);
};


}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__ERROR_CODE_HPP_
