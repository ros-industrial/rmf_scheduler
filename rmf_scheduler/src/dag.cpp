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

#include <ostream>

#include "rmf_scheduler/dag.hpp"
#include "taskflow/taskflow.hpp"

namespace rmf_scheduler
{

DAG::DAG()
: taskflow_(std::make_unique<tf::Taskflow>())
{
}

DAG::~DAG()
{
}

DAG::DAG(DAG && rhs)
{
  taskflow_ = std::move(rhs.taskflow_);
  node_list_ = std::move(rhs.node_list_);
}

DAG & DAG::operator=(DAG && rhs)
{
  taskflow_ = std::move(rhs.taskflow_);
  node_list_ = std::move(rhs.node_list_);
  return *this;
}

DAG::DAG(
  const DAG::Description & description,
  bool safe)
: taskflow_(std::make_unique<tf::Taskflow>())
{
  for (auto & itr : description) {
    add_node(itr.first);
  }

  for (auto & itr : description) {
    add_dependency(itr.first, itr.second, safe);
  }
}

void DAG::add_node(
  const std::string & id)
{
  if (has_node(id)) {
    throw DAGIDException(id.c_str(), "add_node: DAG node [%s] already exists", id.c_str());
  }

  tf::Task task = taskflow_->placeholder();
  node_list_.emplace(id, std::make_unique<tf::Task>(task.name(id)));
}

void DAG::delete_node(
  const std::string & id)
{
  auto itr = node_list_.find(id);
  if (itr == node_list_.end()) {
    return;
  }

  taskflow_->erase(*itr->second);
  node_list_.erase(itr);
}

void DAG::add_dependency(
  const std::string & id,
  const DAG::DependencyInfo & dependency_info,
  bool safe)
{
  // Check if node id exists
  auto node_itr = node_list_.find(id);
  if (node_itr == node_list_.end()) {
    throw DAGIDException(
            id.c_str(), "add_dependency: DAG node [%s] doesn't exist", id.c_str());
  }

  // Check if dependent nodes exist
  std::unordered_map<std::string, tf::Task> dep_nodes;
  for (auto & deps_id : dependency_info) {
    auto deps_itr = node_list_.find(deps_id);
    if (deps_itr == node_list_.end()) {
      throw DAGIDException(
              id.c_str(),
              "add_dependency: DAG dependant node [%s] doesn't exist", deps_id.c_str());
    }
    dep_nodes.emplace(deps_id, *deps_itr->second);
  }

  // Check cyclic if safe is set to true
  if (safe) {
    if (is_cyclic()) {
      throw DAGCyclicException(
              "add_dependency: DAG graph already has cyclic patterns");
    }

    temp_description_info_.emplace(id, dependency_info);
    if (is_cyclic()) {
      temp_description_info_.clear();
      throw DAGCyclicException(
              "add_dependency: New dependency for [%s] causes cyclic patterns", id.c_str());
    }
  }

  // Check if node is already dependent on the target
  node_itr->second->for_each_dependent(
    [&dep_nodes](tf::Task dependent) {
      auto deps_itr = dep_nodes.find(dependent.name());
      if (deps_itr != dep_nodes.end()) {
        dep_nodes.erase(deps_itr);
      }
    });

  for (auto & dep_node : dep_nodes) {
    node_itr->second->succeed(dep_node.second);
  }
}

void DAG::clear_dependency(const std::string & id)
{
  // Get all successor
  auto itr = node_list_.find(id);
  if (itr == node_list_.end()) {
    return;
  }

  std::vector<tf::Task> successors;
  itr->second->for_each_successor(
    [&successors](tf::Task successor) mutable {
      successors.push_back(successor);
    });

  delete_node(id);
  add_node(id);

  // Re-establish the dependencies with the successors
  for (auto & successor : successors) {
    successor.succeed(*node_list_[id]);
  }
}

bool DAG::has_node(const std::string & id) const
{
  return node_list_.find(id) != node_list_.end();
}

bool DAG::is_cyclic() const
{
  // Mark all the vertices as not visited
  // and not part of recursion stack
  std::unordered_map<std::string, bool> visited, rec_stack;
  for (auto & itr : node_list_) {
    visited[itr.first] = false;
    rec_stack[itr.first] = false;
  }


  // Call the recursive helper function
  // to detect cycle in different DFS trees
  for (auto & itr : node_list_) {
    if (!visited.at(itr.first) &&
      _is_cyclic_until(itr.first, visited, rec_stack))
    {
      return true;
    }
  }
  return false;
}

bool DAG::_is_cyclic_until(
  const std::string & id,
  std::unordered_map<std::string, bool> & visited,
  std::unordered_map<std::string, bool> & rec_stack) const
{
  if (!visited.at(id)) {
    // Mark the current node as visited
    // and part of recursion stack
    visited[id] = true;
    rec_stack[id] = true;

    bool is_cyclic = false;
    // Recur for all the vertices adjacent to this
    // vertex
    node_list_.at(id)->for_each_successor(
      [this, &is_cyclic, &visited, &rec_stack](tf::Task successor) {
        std::string successor_id = successor.name();
        if (!visited[successor_id] &&
        _is_cyclic_until(successor_id, visited, rec_stack))
        {
          is_cyclic = true;
        } else if (rec_stack[successor_id]) {
          is_cyclic = true;
        }
      }
    );

    // Also check for the temporary description
    for (auto & temp_deps_itr : temp_description_info_) {
      std::string successor_id = temp_deps_itr.first;
      for (auto & dependent_id : temp_deps_itr.second) {
        if (dependent_id != id) {
          continue;
        }

        if (!visited[successor_id] &&
          _is_cyclic_until(successor_id, visited, rec_stack))
        {
          is_cyclic = true;
          break;
        } else if (rec_stack[successor_id]) {
          is_cyclic = true;
          break;
        }
      }
    }

    if (is_cyclic) {
      return true;
    }
  }

  // Remove the vertex from recursion stack
  rec_stack[id] = false;
  return false;
}

std::vector<std::string> DAG::entry_nodes() const
{
  std::vector<std::string> result_ids;
  taskflow_->for_each_task(
    [&result_ids](tf::Task task) {
      if (task.num_dependents() == 0) {
        result_ids.push_back(task.name());
      }
    });
  return result_ids;
}

std::vector<std::string> DAG::end_nodes() const
{
  std::vector<std::string> result_ids;
  taskflow_->for_each_task(
    [&result_ids](tf::Task task) {
      if (task.num_successors() == 0) {
        result_ids.push_back(task.name());
      }
    });
  return result_ids;
}

std::vector<std::string> DAG::all_nodes() const
{
  std::vector<std::string> result_ids;
  result_ids.reserve(taskflow_->num_tasks());
  taskflow_->for_each_task(
    [&result_ids](tf::Task task) {
      result_ids.push_back(task.name());
    });
  return result_ids;
}

std::string DAG::dot() const
{
  std::ostringstream oss;
  taskflow_->dump(oss);
  return oss.str();
}

DAG::Description DAG::description() const
{
  Description out;
  for (const auto & itr : node_list_) {
    DependencyInfo deps;
    itr.second->for_each_dependent(
      [&deps](tf::Task dependent) mutable {
        deps.push_back(dependent.name());
      });
    out[itr.first] = deps;
  }
  return out;
}

DAG::DependencyInfo DAG::get_dependency_info(const std::string & id) const
{
  DependencyInfo out;
  auto itr = node_list_.find(id);
  if (itr == node_list_.end()) {
    return out;
  }

  itr->second->for_each_dependent(
    [&out](tf::Task dependent) mutable {
      out.push_back(dependent.name());
    });
  return out;
}

}  // namespace rmf_scheduler
