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
#include <sstream>
#include <cassert>
#include "rmf_scheduler/data/dag.hpp"
#include "rmf_scheduler/utils/dag_bt_converter.hpp"

namespace rmf_scheduler
{

namespace data
{
DAG::DAG() {}

DAG::~DAG()
{
}

DAG::DAG(
  const DAG::Description & description,
  bool safe)
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
    throw exception::DAGIDException(
            id.c_str(),
            "add_node: DAG node [%s] already exists", id.c_str());
  }
  graph_.add_node(id);
}

void DAG::update_node(
  const std::string & id,
  const std::string & new_id)
{
  if (!has_node(id)) {
    throw exception::DAGIDException(
            id.c_str(),
            "update_node: DAG node [%s] doesn't exists", id.c_str());
  }
  graph_.update_node(id, new_id);
}

void DAG::delete_node(const std::string & id)
{
  graph_.delete_node(id);
}

void DAG::add_dependency(
  const std::string & id,
  const DAG::DependencyInfo & dependency_info,
  bool safe)
{
  // Check if node id exists
  if (!has_node(id)) {
    throw exception::DAGIDException(
            id.c_str(), "add_dependency: DAG node [%s] doesn't exist", id.c_str());
  }

  // Check if dependent nodes exist
  for (auto & deps_id : dependency_info) {
    if (!has_node(deps_id)) {
      throw exception::DAGIDException(
              id.c_str(),
              "add_dependency: DAG dependant node [%s] doesn't exist", deps_id.c_str());
    }
  }

  // Check cyclic if safe is set to true
  if (safe) {
    if (is_cyclic()) {
      throw exception::DAGCyclicException(
              "add_dependency: DAG graph already has cyclic patterns");
    }

    temp_description_info_.emplace(id, dependency_info);
    if (is_cyclic()) {
      temp_description_info_.clear();
      throw exception::DAGCyclicException(
              "add_dependency: New dependency for [%s] causes cyclic patterns", id.c_str());
    }
  }

  // Check if node is already dependent on the target
  const auto & dependencies = graph_.get_node(id)->inbound_edges();
  for (auto & deps_id : dependency_info) {
    if (dependencies.find(deps_id) != dependencies.end()) {
      continue;
    }
    graph_.add_edge(deps_id, id);
  }
}

void DAG::clear_dependency(const std::string & id)
{
  if (!has_node(id)) {
    return;
  }
  std::unordered_set<std::string> inbound_edges = graph_.get_node(id)->inbound_edges();
  for (auto & source_id : inbound_edges) {
    graph_.delete_edge(source_id, id);
  }
}

bool DAG::has_node(const std::string & id) const
{
  return graph_.has_node(id);
}

bool DAG::is_cyclic() const
{
  // Mark all the vertices as not visited
  // and not part of recursion stack
  std::unordered_map<std::string, bool> visited, rec_stack;
  auto node_list = graph_.get_all_nodes();
  for (auto & itr : node_list) {
    visited[itr.first] = false;
    rec_stack[itr.first] = false;
  }

  // Call the recursive helper function
  // to detect cycle in different DFS trees
  for (auto & itr : node_list) {
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

    // Recur for all the vertices adjacent to this
    // vertex
    std::vector<std::string> dependents = get_dependency_info(id);
    for (auto & dependent_id : dependents) {
      if (!visited.at(dependent_id) && _is_cyclic_until(dependent_id, visited, rec_stack)) {
        return true;
      } else if (rec_stack[dependent_id]) {
        return true;
      }
    }

    auto temp_deps_itr = temp_description_info_.find(id);
    if (temp_deps_itr != temp_description_info_.end()) {
      for (auto & dependent_id : temp_deps_itr->second) {
        if (!visited[dependent_id] &&
          _is_cyclic_until(dependent_id, visited, rec_stack))
        {
          return true;
        } else if (rec_stack[dependent_id]) {
          return true;
        }
      }
    }
  }
  rec_stack[id] = false;
  return false;
}


std::unordered_set<std::string> DAG::entry_nodes() const
{
  std::unordered_set<std::string> result_ids;
  graph_.for_each_node(
    [&result_ids](Node::ConstPtr node) {
      if (node->inbound_edges().empty()) {
        result_ids.emplace(node->id());
      }
    });
  return result_ids;
}

std::unordered_set<std::string> DAG::end_nodes() const
{
  std::unordered_set<std::string> result_ids;
  graph_.for_each_node(
    [&result_ids](Node::ConstPtr node) {
      if (node->outbound_edges().empty()) {
        result_ids.emplace(node->id());
      }
    });
  return result_ids;
}

std::vector<std::string> DAG::all_nodes() const
{
  std::vector<std::string> result_ids;
  graph_.for_each_node(
    [&result_ids](Node::ConstPtr node) {
      result_ids.push_back(node->id());
    }
  );
  return result_ids;
}

std::string DAG::dot() const
{
  std::ostringstream oss;
  graph_.dump(oss);
  return oss.str();
}

DAG::Description DAG::description() const
{
  Description out;
  graph_.for_each_node(
    [&out](Node::ConstPtr node) {
      out[node->id()] = DependencyInfo{
        node->inbound_edges().begin(),
        node->inbound_edges().end()
      };
    }
  );
  return out;
}

DAG::DependencyInfo DAG::get_dependency_info(const std::string & id) const
{
  if (!has_node(id)) {
    return {};
  }

  const auto & dependencies = graph_.get_node(id)->inbound_edges();
  return {
    dependencies.begin(),
    dependencies.end()
  };
}

std::string DAG::generate_bt_xml(const std::string & bt_id, const TaskInfo & callback_func)
{
  auto tasks = generate_ordered_tree();
  // std::cout << tasks << std::endl;
  bt_converter::BTConverter xml(tasks);
  xml.generate_tree(bt_id, callback_func);
  return xml.get_xml_str();
}

std::string DAG::generate_ordered_tree()
{
  bool if_task_before = false;
  bool is_parallel = false;
  std::unordered_map<std::string, bool> visited;
  // stack for keeping track of nodes in parallel for string formatting
  std::stack<size_t> parallel_count_stack;
  size_t parallel_count{0};
  // check to see if what was stored previously in oss was a symbol or task
  // check to see if the current recursion call is a node in parallel sequence
  std::ostringstream oss;
  auto node_list = graph_.get_all_nodes();
  for (auto & node : node_list) {
    visited[node.first] = false;
  }
  auto entry_nodes_ = entry_nodes();
  // TODO(DillonChew98) implement check for null node and removal of null node
  if (entry_nodes_.size() > 1) {
    assert(false);
    add_node("null");
    _get_tree(
      "null", visited, if_task_before, is_parallel, oss, parallel_count,
      parallel_count_stack);
  } else {
    _get_tree(
      *(entry_nodes_.begin()), visited, if_task_before, is_parallel, oss, parallel_count,
      parallel_count_stack);
  }
  return oss.str();
}

void DAG::_get_tree(
  const std::string & id, std::unordered_map<std::string, bool> & visited,
  bool & if_task_before, bool & is_parallel, std::ostream & oss, size_t & parallel_count,
  std::stack<size_t> & parallel_count_stack)
{
  visited[id] = true;
  auto successor_list = graph_.get_node(id)->outbound_edges();
  auto dependency_list = graph_.get_node(id)->inbound_edges();
  // return if there are still dependencies to prevent duplication of node
  for (auto it = dependency_list.begin(); it != dependency_list.end(); ++it) {
    if (!visited[*it]) {
      return;
    }
  }
  // if there are no successors, it is an end node, sets the current node as an action in a sequence
  if (successor_list.empty()) {
    if (if_task_before) {
      oss << ", ";
    }
    oss << "-> " << id << " ";
    if_task_before = true;
  } else if (successor_list.size() == 1) {  // if parent node only has one child
    oss << "-> " << id << " ";
    if_task_before = true;
    is_parallel = false;
    _get_tree(
      *(successor_list.begin()), visited, if_task_before, is_parallel, oss, parallel_count,
      parallel_count_stack);
  } else if (successor_list.size() > 1) {  // branching node, has multiple children
    oss << "-> " << id << " ";
    ++parallel_count;
    parallel_count_stack.push(parallel_count);
    is_parallel = true;
    for (auto & successor : successor_list) {
      for (size_t i{0}; i < parallel_count_stack.top(); ++i) {
        oss << "/";
      }
      if_task_before = false;
      _get_tree(
        successor, visited, if_task_before, is_parallel, oss, parallel_count,
        parallel_count_stack);
      if_task_before = true;
    }
    parallel_count_stack.pop();
  }
}
}  // namespace data

}  // namespace rmf_scheduler
