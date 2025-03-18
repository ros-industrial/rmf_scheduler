// Copyright 2023-2024 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__UTILS__DAG_HELPER_HPP_
#define RMF2_SCHEDULER__UTILS__DAG_HELPER_HPP_

#include <string>
#include <unordered_map>

#include "rmf2_scheduler/data/task.hpp"
#include "rmf2_scheduler/data/graph.hpp"


namespace rmf2_scheduler
{
namespace utils
{

/// function to check whether a graph is a valid DAG
/**
 * \returns False if the graph contains cycle(s)
 */
bool is_valid_dag(const data::Graph & graph);

void update_dag_start_time(
  const data::Graph & graph,
  std::unordered_map<std::string, data::Event::Ptr> & event_map
);

void update_dag_start_time(
  const data::Graph & graph,
  std::unordered_map<std::string, data::Task::Ptr> & task_map
);

}  // namespace utils
}  // namespace rmf2_scheduler
#endif  // RMF2_SCHEDULER__UTILS__DAG_HELPER_HPP_
