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

#ifndef RMF_SCHEDULER__PARSER_HPP_
#define RMF_SCHEDULER__PARSER_HPP_

#include <string>
#include <unordered_map>

#include "nlohmann/json.hpp"
#include "rmf_scheduler/schedule.hpp"

namespace rmf_scheduler
{

namespace parser
{

void json_to_events(
  const nlohmann::json & events_json,
  std::unordered_map<std::string, Event> & events_description);


void json_to_dependencies(
  const nlohmann::json & dependencies_json,
  std::unordered_map<std::string, DAG::Description> & dependencies_description);

void json_to_series_map(
  const nlohmann::json & series_json,
  std::unordered_map<std::string, Series::Description> & series_map_description);

void json_to_schedule(
  const nlohmann::json & schedule_json,
  Schedule::Description & schedule_description);

void events_to_json(
  const std::unordered_map<std::string, Event> & events_description,
  nlohmann::json & events_json);

void dependencies_to_json(
  const std::unordered_map<std::string, DAG::Description> & dependencies_description,
  nlohmann::json & dependencies_json);

void series_map_to_json(
  const std::unordered_map<std::string, Series::Description> & series_map_description,
  nlohmann::json & series_map_json);

void schedule_to_json(
  const Schedule::Description & schedule_description,
  nlohmann::json & schedule_json);

}  // namespace parser

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__PARSER_HPP_
