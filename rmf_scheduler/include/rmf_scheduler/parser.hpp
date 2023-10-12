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
#include <vector>

#include "nlohmann/json.hpp"
#include "rmf_scheduler/data/schedule.hpp"

namespace rmf_scheduler
{

namespace parser
{

void json_to_events(
  const nlohmann::json & events_json,
  std::unordered_map<std::string, data::Event> & events_description);


void json_to_dependencies(
  const nlohmann::json & dependencies_json,
  std::unordered_map<std::string, data::DAG::Description> & dependencies_description);

void json_to_series_map(
  const nlohmann::json & series_json,
  std::unordered_map<std::string, data::Series::Description> & series_map_description);

void json_to_schedule(
  const nlohmann::json & schedule_json,
  data::Schedule::Description & schedule_description);

void json_to_update_event_time(
  const nlohmann::json & update_event_time_json,
  data::Event & update_event_time);

void json_to_update_series_map(
  const nlohmann::json & update_series_request_json,
  std::unordered_map<std::string, data::Series::Update> & update_series_map);

void events_to_json(
  const std::unordered_map<std::string, data::Event> & events_description,
  nlohmann::json & events_json,
  bool full = false);

void dependencies_to_json(
  const std::unordered_map<std::string, data::DAG::Description> & dependencies_description,
  nlohmann::json & dependencies_json);

void series_map_to_json(
  const std::unordered_map<std::string, data::Series::Description> & series_map_description,
  nlohmann::json & series_map_json);

void schedule_to_json(
  const data::Schedule::Description & schedule_description,
  nlohmann::json & schedule_json,
  bool full = false);

template<typename DetailT>
bool filter_event_details(
  const nlohmann::json & event_details_json,
  const std::string & filter_descriptor,
  DetailT & filtered_detail,
  const std::string & delimiter = "::")
{
  // Use find function to find 1st position of delimiter.
  size_t start = 0;
  size_t end = filter_descriptor.find(delimiter);

  std::string field_descriptor = filter_descriptor.substr(start, end - start);
  nlohmann::json::const_iterator itr = event_details_json.find(field_descriptor);
  if (itr == event_details_json.end()) {
    return false;
  }
  while (end != std::string::npos) {  // Loop until no delimiter is left in the string.
    start = end + delimiter.size();
    end = filter_descriptor.find(delimiter, start);
    size_t len = end == std::string::npos ? end : end - start;
    // Iterate one more time
    field_descriptor = filter_descriptor.substr(start, len);
    auto new_itr = itr->find(field_descriptor);
    if (new_itr == itr->end()) {
      return false;
    } else {
      itr = new_itr;
    }
  }

  // Return raw JSON is it is specified as the type
  if constexpr (std::is_same<DetailT, nlohmann::json>::value) {
    filtered_detail = *itr;
    return true;
  }

  try {
    filtered_detail = itr->get<DetailT>();
  } catch (const std::exception & e) {
    return false;
  }
  return true;
}

template<typename DetailT>
bool filter_event_details(
  const std::string & event_details,
  const std::string & filter_descriptor,
  DetailT & filtered_detail,
  const std::string & delimiter = "::")
{
  nlohmann::json event_details_json;
  try {
    event_details_json = nlohmann::json::parse(event_details);
  } catch (const std::exception & e) {
    return false;
  }
  return filter_event_details<DetailT>(
    event_details_json, filter_descriptor, filtered_detail, delimiter);
}

template<typename DetailT>
void batch_filter_event_details(
  const std::string & event_details,
  const std::vector<std::string> & filter_descriptors,
  std::vector<bool> & filter_success,
  std::vector<DetailT> & filtered_details,
  const std::string & delimiter = "::")
{
  nlohmann::json event_details_json;
  filter_success.clear();
  filter_success.resize(filter_descriptors.size(), false);
  filtered_details.clear();
  filtered_details.resize(filter_descriptors.size());
  try {
    event_details_json = nlohmann::json::parse(event_details);
  } catch (const std::exception & e) {
    return;
  }
  for (size_t i = 0; i < filter_descriptors.size(); i++) {
    filter_success[i] = filter_event_details<DetailT>(
      event_details_json, filter_descriptors[i], filtered_details[i], delimiter);
  }
}

}  // namespace parser

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__PARSER_HPP_
