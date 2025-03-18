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

#include "rmf2_scheduler/utils/dag_helper.hpp"
#include "taskflow/taskflow.hpp"

namespace rmf2_scheduler
{
namespace utils
{

namespace
{

// helper function to check whether a graph contains cycle(s)
bool _is_cyclic_util(
  const std::string & id,
  std::unordered_map<std::string, bool> & visited,
  std::unordered_map<std::string, bool> & rec_stack,
  std::unordered_map<std::string, data::Node::ConstPtr> node_list)
{
  if (!visited.at(id)) {
    // Mark the current node as visited
    // and part of recursion stack
    visited[id] = true;
    rec_stack[id] = true;

    // Recur for all the vertices adjacent to this vertex
    for (const auto & [dependent_id, edge] : node_list.at(id)->inbound_edges()) {
      if (!visited.at(dependent_id) &&
        _is_cyclic_util(dependent_id, visited, rec_stack, node_list))
      {
        return true;
      } else if (rec_stack.at(dependent_id)) {
        return true;
      }
    }
  }
  rec_stack[id] = false;
  return false;
}
}  // namespace

bool is_valid_dag(const data::Graph & graph)
{
  // Mark all the vertices as not visited
  // and not part of recursion stack
  std::unordered_map<std::string, bool> visited, rec_stack;
  auto node_list = graph.get_all_nodes();
  for (const auto & [id, node] : node_list) {
    visited[id] = false;
    rec_stack[id] = false;
  }

  // Call the recursive helper function
  // to detect cycle in different DFS trees
  for (const auto & [id, node] : node_list) {
    if (!visited.at(id) &&
      _is_cyclic_util(id, visited, rec_stack, node_list))
    {
      return false;
    }
  }
  return true;
}

namespace
{

void _update_node_start_time(
  const data::Node::ConstPtr & node,
  std::unordered_map<std::string, data::Event::Ptr> & event_map,
  std::unordered_map<std::string, data::Time> & calculated_end_time
)
{
  auto event_itr = event_map.find(node->id());
  assert(event_itr != event_map.end());
  auto event = event_itr->second;
  auto dependents = node->inbound_edges();
  if (!dependents.empty()) {
    auto dependent_event_itr = event_map.find(dependents.begin()->first);
    data::Time max_start_time = dependent_event_itr->second->start_time;
    data::Edge max_edge = dependents.begin()->second;

    for (auto & itr : dependents) {
      auto end_time_itr = calculated_end_time.find(itr.first);
      assert(end_time_itr != calculated_end_time.end());
      data::Time end_time = end_time_itr->second;
      if (end_time > max_start_time) {
        max_start_time = end_time;
        max_edge = itr.second;
      }
    }

    if (event->start_time < max_start_time || max_edge.type == "hard") {
      event->start_time = max_start_time;
    }
  }

  calculated_end_time[node->id()] = event->start_time +
    (event->duration.has_value() ? *event->duration : data::Duration(0));
}

}  // namespace

void update_dag_start_time(
  const data::Graph & graph,
  std::unordered_map<std::string, data::Event::Ptr> & event_map
)
{
  std::unordered_map<std::string, data::Time> calculated_end_time;
  auto work_fn = std::bind(
    &_update_node_start_time,
    std::placeholders::_1,
    std::ref(event_map),
    std::ref(calculated_end_time)
  );

  // Create Taskflow for execution
  tf::Taskflow taskflow;
  std::unordered_map<std::string, tf::Task> task_map;
  graph.for_each_node(
    [work_fn, &taskflow, &task_map](const data::Node::ConstPtr & node) {
      task_map[node->id()] = taskflow.emplace(
        [work_fn, node]() {
          work_fn(node);
        });
    });

  graph.for_each_node(
    [&task_map](const data::Node::ConstPtr & node) {
      auto dependents = node->inbound_edges();
      for (auto & dependent_itr : dependents) {
        task_map[node->id()].succeed(task_map[dependent_itr.first]);
      }
    });

  // Execute
  // Single thread is good enough
  tf::Executor executor(1);
  executor.run(taskflow).wait();
  // TODO(anyone): safe exit based on visit counts
}

void update_dag_start_time(
  const data::Graph & graph,
  std::unordered_map<std::string, data::Task::Ptr> & task_map
)
{
  std::unordered_map<std::string, data::Event::Ptr> event_map{task_map.begin(), task_map.end()};
  update_dag_start_time(graph, event_map);
}
}  // namespace utils
}  // namespace rmf2_scheduler
