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

#include <cassert>
#include <ostream>

#include "rmf_scheduler/data/graph.hpp"

namespace rmf_scheduler
{

namespace data
{

Graph::Graph()
{
}

Graph::~Graph()
{
}

void Graph::add_node(const std::string & id)
{
  // Check new ID against old ID
  assert(!has_node(id));

  nodes_.emplace(id, std::make_shared<Node>(id));
}

void Graph::add_edge(
  const std::string & source,
  const std::string & destination
)
{
  // Check source and destination exists
  assert(has_node(source));
  assert(has_node(destination));

  // Check edges doesn't exist
  assert(
    nodes_[source]->outbound_edges().find(destination) ==
    nodes_[source]->outbound_edges().end()
  );

  Node::Restricted token;
  nodes_[source]->outbound_edges(token).emplace(destination);
  nodes_[destination]->inbound_edges(token).emplace(source);
}

void Graph::delete_edge(
  const std::string & source,
  const std::string & destination
)
{
  // Check source and destination exists
  assert(has_node(source));
  assert(has_node(destination));

  // Check edges exist
  assert(
    nodes_[source]->outbound_edges().find(destination) !=
    nodes_[source]->outbound_edges().end()
  );

  Node::Restricted token;
  nodes_[source]->outbound_edges(token).erase(destination);
  nodes_[destination]->inbound_edges(token).erase(source);
}

void Graph::update_node(
  const std::string & id,
  const std::string & new_id
)
{
  // Check new ID against old ID
  assert(new_id != id);

  Node::Restricted token;
  auto itr = nodes_.find(id);

  // Check node exist
  assert(itr != nodes_.end());

  // Create a new node
  Node::Ptr new_node = std::make_shared<Node>(new_id);
  for (auto & source_id : itr->second->inbound_edges()) {
    // Rename source node outbound edges
    Node::Ptr source_node = nodes_[source_id];
    source_node->outbound_edges(token).erase(id);
    source_node->outbound_edges(token).emplace(new_id);

    // add inbound edges to the new node
    new_node->inbound_edges(token).emplace(source_id);
  }

  for (auto & destination_id : itr->second->outbound_edges()) {
    // Rename destination node inbound edges
    Node::Ptr destination_node = nodes_[destination_id];
    destination_node->inbound_edges(token).erase(id);
    destination_node->inbound_edges(token).emplace(new_id);

    // add inbound edges to the new node
    new_node->outbound_edges(token).emplace(destination_id);
  }

  nodes_.erase(itr);
  nodes_[new_id] = new_node;
}

void Graph::delete_node(const std::string & id)
{
  Node::Restricted token;
  auto itr = nodes_.find(id);

  // Check new ID against old ID
  assert(itr != nodes_.end());

  // Erase inbound edges
  for (auto & source_id : itr->second->inbound_edges()) {
    nodes_[source_id]->outbound_edges(token).erase(id);
  }

  // Erase outbound edges
  for (auto & destination_id : itr->second->outbound_edges()) {
    nodes_[destination_id]->inbound_edges(token).erase(id);
  }

  nodes_.erase(itr);
}

bool Graph::has_node(const std::string & id) const
{
  return nodes_.find(id) != nodes_.end();
}

Node::ConstPtr Graph::get_node(const std::string & id) const
{
  assert(has_node(id));

  return nodes_.at(id);
}

std::unordered_map<std::string, Node::ConstPtr>
Graph::get_all_nodes() const
{
  return {nodes_.begin(), nodes_.end()};
}

void Graph::dump(std::ostream & oss) const
{
  std::vector<std::string> edges;
  oss << "digraph DAG {\n";
  for (auto & itr : nodes_) {
    for (auto & destination_id : itr.second->outbound_edges()) {
      oss << itr.first << " -> ";
      oss << destination_id << ";" << std::endl;
    }
  }
  oss << "}" << std::endl;
}

}  // namespace data

}  // namespace rmf_scheduler
