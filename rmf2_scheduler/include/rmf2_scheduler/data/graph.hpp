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

#ifndef RMF2_SCHEDULER__DATA__GRAPH_HPP_
#define RMF2_SCHEDULER__DATA__GRAPH_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <map>
#include <vector>

#include "rmf2_scheduler/data/node.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace data
{

class Graph
{
public:
  RS_SMART_PTR_DEFINITIONS(Graph)

  /// Constructor
  Graph();

  /// Destructor
  virtual ~Graph();

  // Special copy constructor
  Graph(const Graph &);
  Graph & operator=(const Graph &);

  // movable
  Graph(Graph &&) = default;
  Graph & operator=(Graph &&) = default;

  /// Add node
  /**
   * \throws std::invalid_argument if node ID already exists
   */
  void add_node(const std::string & id);

  /// Add edge
  /**
   * \param source source node ID
   * \param destination destination node ID
   * \param edge edge property
   * \throws std::out_of_range if source or destination node doesn't exist
   * \throws std::invalid_argument if edge already exists
   */
  void add_edge(
    const std::string & source,
    const std::string & destination,
    const Edge & edge = Edge()
  );

  /// Update node id
  /**
   * \throws std::out_of_range if node doesn't exist
   * \throws std::invalid_argument if new node ID already exists or same as the old node ID.
   */
  void update_node(
    const std::string & id,
    const std::string & new_id
  );

  /// Delete edge
  /**
   * \param source source node ID
   * \param destination destination node ID
   * \throws std::out_of_range if source, destination node or edge doesn't exist
   */
  void delete_edge(
    const std::string & source,
    const std::string & destination
  );

  /// Delete node
  /**
   * \throws std::out_of_range if node doesn't exist
   */
  void delete_node(const std::string & id);

  /// Prune
  /**
   * Delete unconnected nodes
   */
  void prune();

  /// For each node
  template<typename V>
  void for_each_node(V && visitor) const;

  /// For each node in order
  template<typename V>
  void for_each_node_ordered(V && visitor) const;

  /// Check if node exist
  bool has_node(const std::string & id) const;

  /// Get node
  /**
   * \throws std::out_of_range if node doesn't exist
   */
  Node::ConstPtr get_node(const std::string & id) const;

  /// Get all nodes
  std::unordered_map<std::string, Node::ConstPtr> get_all_nodes() const;

  /// Get all nodes in order
  std::map<std::string, Node::ConstPtr> get_all_nodes_ordered() const;

  [[nodiscard]] bool empty() const;

  [[nodiscard]] std::size_t size() const;

  /// Generate DOT graph
  void dump(std::ostream & oss) const;

  inline bool operator==(const Graph & rhs) const
  {
    if (this->nodes_.size() != rhs.nodes_.size()) {return false;}
    for (auto & itr : this->nodes_) {
      const auto rhs_itr = rhs.nodes_.find(itr.first);
      if (rhs_itr == rhs.nodes_.end()) {return false;}
      if (*itr.second != *rhs_itr->second) {return false;}
    }
    return true;
  }

  inline bool operator!=(const Graph & rhs) const
  {
    return !(*this == rhs);
  }

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

template<typename V>
void Graph::for_each_node_ordered(V && visitor) const
{
  std::map<std::string, Node::Ptr> ordered_nodes{nodes_.begin(), nodes_.end()};
  for (auto & itr : ordered_nodes) {
    visitor(itr.second);
  }
}


}  // namespace data

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__DATA__GRAPH_HPP_
