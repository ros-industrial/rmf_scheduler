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

#include "rmf_scheduler/schema_validator.hpp"

#include <functional>
#include <iostream>

namespace rmf_scheduler
{

SchemaValidator::SchemaValidator()
{
}

SchemaValidator::SchemaValidator(
  const std::vector<nlohmann::json> & schemas)
{
  add_schemas(schemas);
}

void SchemaValidator::add_schemas(
  const std::vector<nlohmann::json> & schemas)
{
  std::lock_guard<std::mutex> lk(mtx_);
  for (auto & schema : schemas) {
    const auto json_uri = nlohmann::json_uri{schema["$id"]};
    schema_lookup_.emplace(json_uri.url(), schema);
    nlohmann::json_schema::json_validator validator(
      schema,
      std::bind(
        &SchemaValidator::schema_loader,
        this,
        std::placeholders::_1,
        std::placeholders::_2)
    );
    validators_.emplace(
      json_uri.url(),
      std::move(validator));
  }
}

void SchemaValidator::validate(
  const nlohmann::json_uri & id,
  const nlohmann::json & json) const
{
  validators_.at(id.url()).validate(json);
}

void SchemaValidator::schema_loader(
  const nlohmann::json_uri & id,
  nlohmann::json & value)
{
  value = schema_lookup_[id.url()];
}

}  // namespace rmf_scheduler
