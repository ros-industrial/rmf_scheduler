// Copyright 2024 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__DATA__JSON_MACROS_HPP_
#define RMF2_SCHEDULER__DATA__JSON_MACROS_HPP_

#include "nlohmann/json.hpp"

/// De-serialize optional members
#define RS_JSON_FROM_WITH_DEFAULT(v1) \
  if (nlohmann_json_j.contains(#v1)) { \
    NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_FROM(v1)) \
  }

/// Define arbitratry class with required members
/**
 *
 * For a struct like the following
 *
 * ```cpp
 * namespace ns
 * {
 *
 * struct TestStruct {
 *   std::string id;
 *   int index;
 * };
 *
 * }  // namespace ns
 * ```
 *
 * Define the serializer and deserializer as follows.
 *
 * ```cpp
 * namespace ns
 * {
 *
 * RS_JSON_DEFINE(TestStruct,
 *   id,
 *   index
 * )
 *
 * }  // namespace ns
 * ```
 *
 */
#define RS_JSON_DEFINE(Type, ...) \
  NLOHMANN_JSON_EXPAND(NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Type, __VA_ARGS__))

/// Define JSON serializer `to_json` only.
#define RS_JSON_SERIALIZER_DEFINE_TYPE(Type, ...) \
  inline void to_json(nlohmann::json & nlohmann_json_j, const Type & nlohmann_json_t) { \
    NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(NLOHMANN_JSON_TO, __VA_ARGS__)) \
  }

/// Define custom JSON deserializer `from_json` (for optional members).
/**
 *
 * This is intended to use together with
 * - `RS_JSON_DESERIALIZER_REQUIRED_MEMBERS`
 * - `RS_JSON_DESERIALIZER_OPTIONAL_MEMBERS`
 *
 * Below is an example
 * ```cpp
 * namespace ns
 * {
 *
 * RS_JSON_DESERIALIZER_DEFINE_TYPE(TestStructOptional
 *   RS_JSON_DESERIALIZER_REQUIRED_MEMBERS(
 *     id,
 *     index
 *   ),
 *   RS_JSON_DESERIALIZER_OPTIONAL_MEMBERS(
 *     optional_id,
 *     optional_index
 *   )
 * )
 *
 * }  // namespace ns
 * ```
 *
 */
#define RS_JSON_DESERIALIZER_DEFINE_TYPE(Type, ...) \
  inline void from_json(const nlohmann::json & nlohmann_json_j, Type & nlohmann_json_t) { \
    const Type nlohmann_json_default_obj{}; \
    NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(NLOHMANN_JSON_EXPAND, __VA_ARGS__)) \
  }

/// Defined required deserializer members
#define RS_JSON_DESERIALIZER_REQUIRED_MEMBERS(...) \
  NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(NLOHMANN_JSON_FROM, __VA_ARGS__))

/// Defined optional deserializer members
#define RS_JSON_DESERIALIZER_OPTIONAL_MEMBERS(...) \
  NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(RS_JSON_FROM_WITH_DEFAULT, __VA_ARGS__))


#endif  // RMF2_SCHEDULER__DATA__JSON_MACROS_HPP_
