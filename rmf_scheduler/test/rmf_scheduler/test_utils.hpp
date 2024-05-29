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

#ifndef RMF_SCHEDULER__TEST_UTILS_HPP_
#define RMF_SCHEDULER__TEST_UTILS_HPP_

#include <iostream>
#include <random>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "nlohmann/json.hpp"

#include "rmf_scheduler/data/event.hpp"
#include "rmf_scheduler/utils/uuid.hpp"

namespace rmf_scheduler
{

namespace test_utils
{

inline std::vector<rmf_scheduler::data::Event> random_event_generator(
  uint64_t start_time,
  uint64_t end_time,
  int sample_size)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)
  std::vector<data::Event> events;
  std::default_random_engine gen;
  std::uniform_int_distribution<uint64_t> dist(start_time, end_time);
  for (int i = 0; i < sample_size; i++) {
    uint64_t start_time = dist(gen);
    std::ostringstream s;
    s << i << '-' << start_time;
    std::string event_id = s.str();
    events.emplace_back(
      data::Event{
          "Randomly generated time event",
          "robot_task",
          start_time,
          10,
          event_id,
          "",
          "",
          "",
          ""
        });
  }
  return events;
}

inline bool is_event_equal(const data::Event & lhs, const data::Event & rhs)
{
  return
    lhs.description == rhs.description &&
    lhs.type == rhs.type &&
    lhs.start_time == rhs.start_time &&
    lhs.duration == rhs.duration &&
    lhs.id == rhs.id &&
    lhs.series_id == rhs.series_id &&
    lhs.dependency_id == rhs.dependency_id &&
    lhs.event_details == rhs.event_details;
}

inline bool is_event_vector_equal(
  const std::vector<rmf_scheduler::data::Event> & lhs,
  const std::vector<rmf_scheduler::data::Event> & rhs)
{
  if (lhs.size() != rhs.size()) {
    return false;
  }

  bool result = true;
  // Compare the result
  // std::cout << "Result EventsHandler:\n";
  std::unordered_set<std::string> keys;
  for (auto & event : lhs) {
    keys.emplace(event.id);
    // std::cout << event.id << ":\t" << event.start_time << std::endl;
  }

  // std::cout << "\nResult Greedy:\n";
  for (auto & event : rhs) {
    result &= (keys.find(event.id) != keys.end());
    // std::cout << event.id << ":\t" << event.start_time << std::endl;
  }
  return result;
}

/// Only work with hashable standard types
template<typename T>
inline bool is_vector_equal(
  const std::vector<T> & lhs,
  const std::vector<T> & rhs)
{
  if (lhs.size() != rhs.size()) {
    return false;
  }
  std::unordered_multiset<T> s1(lhs.begin(), lhs.end());
  std::unordered_multiset<T> s2(rhs.begin(), rhs.end());

  return s1 == s2;
}

inline double to_ms(uint64_t ns)
{
  return static_cast<double>(ns) / 1e6;
}

// Load a set of robot events that do not clash
inline std::vector<data::Event> load_clashing_events(
  int num_robots,
  int num_location,
  int num_events)
{
  std::vector<data::Event> events;
  std::vector<std::string> locations;
  for (int i = 0; i < num_location; i++) {
    locations.push_back("location_" + std::to_string(i));
  }
  const uint64_t start = 0;
  const uint64_t duration = 60;

  // Add robot task events
  for (int i = 0; i < num_robots; i++) {
    const std::string robot = "robot_" + std::to_string(i);
    for (int j = 0; j < num_events; j++) {
      std::string id = utils::gen_uuid();

      // Generate Event Details
      nlohmann::json event_details_json;
      event_details_json["robot"] = robot;
      // random location so no clashes
      event_details_json["zone"] = locations[j % locations.size()];

      events.push_back(
        data::Event{
            "",                               // description
            "default/robot_task",             // type
            start + (duration / 2) * j,       // start time
            duration,                         // duration
            id,                               // id
            "",                               // series id
            "",                               // resource id
            "",                               // dependency id
            event_details_json.dump()         // event_details
          }
      );
    }
  }

  // Add flight schedule based events
  for (int i = 0; i < num_location; i++) {
    std::string id = utils::gen_uuid();

    // Generate Event Details
    nlohmann::json event_details_json;
    // random location so no clashes
    event_details_json["zone"] = locations[i];
    events.push_back(
      data::Event {
          "",                               // description
          "flight-schedule",                // type
          start + duration * i,             // start time
          duration,                         // duration
          id,                               // id
          "",                               // series id
          "",                               // resource id
          "",                               // dependency id
          event_details_json.dump(),        // event details
        }
    );
  }

  return events;
}

inline std::vector<data::Event> load_no_clashing_events(
  int num_robots,
  int num_events)
{
  std::vector<data::Event> events;
  const uint64_t start = 0;
  const uint64_t duration = 60;
  for (int i = 0; i < num_robots; i++) {   // 2 robots
    const std::string robot = "robot_" + std::to_string(i);
    for (int j = 0; j < num_events; j++) {  // 3 events
      std::string id = utils::gen_uuid();
      std::string event_details = "{robot:" + robot + "}";
      events.push_back(
        data::Event{
            "",                     // description
            "default/robot_task",   // type
            start + duration * j,   // start time
            duration,               // duration
            id,                     // id
            "",                     // series id
            "",                     // resource id
            "",                     // dependency id
            event_details,          // event details
          }
      );
    }
  }
  return events;
}

inline void print_events(const std::vector<data::Event> & events)
{
  for (auto event : events) {
    std::cout << "Event \n"
              << std::endl
              << "\tid: "
              << event.id
              << "\n \tstart time: "
              << event.start_time
              << "\n \tduration: "
              << event.duration
              << "\n \tevent details: "
              << event.event_details
              << std::endl;
  }
}

inline std::string init_json_from_file(
  const std::string & path,
  bool relative = true)
{
  std::ifstream f_json(
    relative ? std::string(TEST_DIRECTORY) + path : path);
  std::stringstream b_json;
  b_json << f_json.rdbuf();
  return b_json.str();
}

}  // namespace test_utils

}  // namespace rmf_scheduler


#endif  // RMF_SCHEDULER__TEST_UTILS_HPP_
