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
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "rmf_scheduler/event.hpp"

namespace rmf_scheduler
{

namespace test_utils
{

inline std::vector<rmf_scheduler::Event> random_event_generator(
  uint64_t start_time,
  uint64_t end_time,
  int sample_size)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)
  std::vector<Event> events;
  std::default_random_engine gen;
  std::uniform_int_distribution<uint64_t> dist(start_time, end_time);
  for (int i = 0; i < sample_size; i++) {
    uint64_t start_time = dist(gen);
    std::ostringstream s;
    s << i << '-' << start_time;
    std::string event_id = s.str();
    events.emplace_back(
      Event{
          "Randomly generated time event",
          "robot_task",
          start_time,
          10,
          event_id,
          "",
          ""
        });
  }
  return events;
}

inline bool is_event_equal(const Event & lhs, const Event & rhs)
{
  return
    lhs.description == rhs.description &&
    lhs.type == rhs.type &&
    lhs.start_time == rhs.start_time &&
    lhs.duration == rhs.duration &&
    lhs.id == rhs.id &&
    lhs.series_id == rhs.series_id &&
    lhs.dag_id == rhs.dag_id;
}

inline bool is_event_vector_equal(
  const std::vector<rmf_scheduler::Event> & lhs,
  const std::vector<rmf_scheduler::Event> & rhs)
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


}  // namespace test_utils

}  // namespace rmf_scheduler


#endif  // RMF_SCHEDULER__TEST_UTILS_HPP_
