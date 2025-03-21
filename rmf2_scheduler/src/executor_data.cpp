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

#include <cstring>

#include "rmf2_scheduler/executor_data.hpp"
#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler
{

void ExecutorData::set_data(const void * data, size_t data_size)
{
  data_.resize(data_size);
  memcpy(data_.data(), data, data_size);
}

void ExecutorData::set_data_as_string(const std::string & data)
{
  set_data(data.c_str(), data.size());
}

void ExecutorData::set_data_as_c_string(const char * data)
{
  set_data(data, strlen(data));
}

void ExecutorData::set_data_as_json(const nlohmann::json & data)
{
  set_data_as_string(data.dump());
}

std::string ExecutorData::get_data_as_string() const
{
  if (data_.empty()) {
    return std::string();
  }
  auto chars = reinterpret_cast<const char *>(data_.data());
  return std::string(chars, data_.size());
}

std::optional<nlohmann::json> ExecutorData::get_data_as_json() const
{
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(get_data_as_string());
  } catch (const std::exception & /*e*/) {
    return std::nullopt;
  }

  return j;
}

}  // namespace rmf2_scheduler
