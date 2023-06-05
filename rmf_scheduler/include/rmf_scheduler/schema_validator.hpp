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

#ifndef RMF_SCHEDULER__SCHEMA_VALIDATOR_HPP_
#define RMF_SCHEDULER__SCHEMA_VALIDATOR_HPP_

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "nlohmann/json.hpp"
#include "nlohmann/json-schema.hpp"

namespace rmf_scheduler
{

class SchemaValidator
{
public:
  SchemaValidator();
  explicit SchemaValidator(const std::vector<nlohmann::json> & schemas);
  ~SchemaValidator() {}

  /// Add a schema to the schema validator
  /**
   * This is thread safe when call from multiple threads.
   * This is not thread safe when call together with validate.
   *
   * \param[in] schemas, schemas to add to the validator
   */
  void add_schemas(const std::vector<nlohmann::json> & schemas);

  void validate(const nlohmann::json_uri & id, const nlohmann::json & json) const;

private:
  void schema_loader(const nlohmann::json_uri & id, nlohmann::json & value);

  std::mutex mtx_;
  std::unordered_map<std::string, nlohmann::json_schema::json_validator> validators_;
  std::unordered_map<std::string, nlohmann::json> schema_lookup_;
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__SCHEMA_VALIDATOR_HPP_
