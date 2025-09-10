// Copyright 2025 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__EXECUTOR_DATA_HPP_
#define RMF2_SCHEDULER__EXECUTOR_DATA_HPP_

#include <string>
#include <vector>
#include <optional>

#include "nlohmann/json.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

class ExecutorData
{
public:
  ExecutorData() = default;
  virtual ~ExecutorData() {}

  /// Add/Retrieve request/response body data.
  void set_data(const void * data, size_t data_size);
  void set_data_as_string(const std::string & data);
  void set_data_as_c_string(const char * data);
  void set_data_as_json(const nlohmann::json & data);

  const std::vector<uint8_t> & get_data() const {return data_;}

  std::string get_data_as_string() const;
  std::optional<nlohmann::json> get_data_as_json() const;
  std::string get_data_as_normalized_json_string() const;

private:
  RS_DISABLE_COPY(ExecutorData)

  // Data buffer.
  std::vector<uint8_t> data_;
};
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__EXECUTOR_DATA_HPP_
