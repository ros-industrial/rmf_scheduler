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

#ifndef RMF_SCHEDULER__DATA__GRAPH_HPP_
#define RMF_SCHEDULER__DATA__GRAPH_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "rmf_scheduler/data/node.hpp"

namespace rmf_scheduler
{

namespace data
{

class Graph
{
public:
  /// Constructor
  Graph();

  /// Destructor
  virtual ~Graph();

  // Not copiable
  Graph(const Graph &) = delete;
  Graph & operator=(const Graph &) = delete;

  // movable
  Graph(Graph &&) = default;
  Graph & operator=(Graph &&) = default;

  /// Add node
  void add_node(const std::string & id);

  /// Add edge
  void add_edge(
    const std::string & source,
    const std::string & destination
  );

  /// Update node id
  void update_node(
    const std::string & id,
    const std::string & new_id
  );

  /// Delete edge
  void delete_edge(
    const std::string & source,
    const std::string & destination
  );

  /// Delete node
  void delete_node(const std::string & id);

  /// For each task
  template<typename V>
  void for_each_node(V && visitor) const;

  /// Check if node exist
  bool has_node(const std::string & id) const;

  /// Get node
  Node::ConstPtr get_node(const std::string & id) const;

  /// Get all nodes
  std::unordered_map<std::string, Node::ConstPtr>
  get_all_nodes() const;

  /// Generate DOT graph
  void dump(std::ostream & oss) const;

private:
  std::unordered_map<std::string, Node::Ptr> nodes_;
};


template<typename V>
void Graph::for_each_node(V && visitor) const
{
  for (auto & itr : nodes_) {
    visitor(itr.second);
  }
}


}  // namespace data

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__DATA__GRAPH_HPP_
