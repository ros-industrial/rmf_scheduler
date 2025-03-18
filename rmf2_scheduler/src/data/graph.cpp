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

#include <map>
#include <stdexcept>
#include <ostream>

#include "rmf2_scheduler/data/graph.hpp"

namespace rmf2_scheduler
{

namespace data
{

Graph::Graph()
{
}

Graph::~Graph()
{
}

Graph::Graph(const Graph & rhs)
{
  for (auto & itr : rhs.nodes_) {
    nodes_.emplace(itr.first, std::make_shared<Node>(*itr.second));
  }
}

Graph & Graph::operator=(const Graph & rhs)
{
  for (auto & itr : rhs.nodes_) {
    nodes_.emplace(itr.first, std::make_shared<Node>(*itr.second));
  }
  return *this;
}

void Graph::add_node(const std::string & id)
{
  // Check new ID against old ID
  if (has_node(id)) {
    throw std::invalid_argument(
            "Graph add_node failed, node [" + id + "] already exists."
    );
  }

  nodes_.emplace(id, std::make_shared<Node>(id));
}

void Graph::add_edge(
  const std::string & source,
  const std::string & destination,
  const Edge & edge
)
{
  // Check source and destination exists
  auto source_itr = nodes_.find(source);
  if (source_itr == nodes_.end()) {
    throw std::out_of_range(
            "Graph add_edge failed, node [" + source + "] doesn't exist."
    );
  }

  auto destination_itr = nodes_.find(destination);
  if (destination_itr == nodes_.end()) {
    throw std::out_of_range(
            "Graph add_edge failed, node [" + destination + "] doesn't exist."
    );
  }

  // Check edges doesn't exist
  if (source_itr->second->outbound_edges().find(destination) !=
    source_itr->second->outbound_edges().end())
  {
    throw std::invalid_argument(
            "Graph add_edge failed, edge "
            "[" + source + "] to [" + destination + "]"
            " already exists."
    );
  }

  Node::Restricted token;
  source_itr->second->outbound_edges(token).emplace(destination, edge);
  destination_itr->second->inbound_edges(token).emplace(source, edge);
}

void Graph::update_node(
  const std::string & id,
  const std::string & new_id
)
{
  // Check new ID against old ID
  if (new_id == id) {
    throw std::invalid_argument(
            "Graph update_node failed, the new ID is the same as the old ID."
    );
  }

  // Check old ID exists
  auto itr = nodes_.find(id);
  if (itr == nodes_.end()) {
    throw std::out_of_range(
            "Graph update_node failed, node [" + id + "] doesn't exist."
    );
  }

  // Check new ID doesn't exist
  if (nodes_.find(new_id) != nodes_.end()) {
    throw std::invalid_argument(
            "Graph update_node failed, new node ID [" + new_id + "] already exists."
    );
  }

  // Create a new node
  Node::Ptr new_node = std::make_shared<Node>(new_id);
  Node::Restricted token;
  for (auto & inbound_edge : itr->second->inbound_edges()) {
    // Rename source node outbound edges
    Node::Ptr source_node = nodes_[inbound_edge.first];
    source_node->outbound_edges(token).erase(id);
    source_node->outbound_edges(token).emplace(new_id, inbound_edge.second);

    // add inbound edges to the new node
    new_node->inbound_edges(token).emplace(inbound_edge);
  }

  for (auto & outbound_edge : itr->second->outbound_edges()) {
    // Rename destination node inbound edges
    Node::Ptr destination_node = nodes_[outbound_edge.first];
    destination_node->inbound_edges(token).erase(id);
    destination_node->inbound_edges(token).emplace(new_id, outbound_edge.second);

    // add inbound edges to the new node
    new_node->outbound_edges(token).emplace(outbound_edge);
  }

  nodes_.erase(itr);
  nodes_[new_id] = new_node;
}

void Graph::delete_edge(
  const std::string & source,
  const std::string & destination
)
{
  // Check source and destination exists
  auto source_itr = nodes_.find(source);
  if (source_itr == nodes_.end()) {
    throw std::out_of_range(
            "Graph delete_edge failed, node [" + source + "] doesn't exist."
    );
  }

  auto destination_itr = nodes_.find(destination);
  if (destination_itr == nodes_.end()) {
    throw std::out_of_range(
            "Graph delete_edge failed, node [" + destination + "] doesn't exist."
    );
  }

  // Check edges exist
  if (source_itr->second->outbound_edges().find(destination) ==
    source_itr->second->outbound_edges().end())
  {
    throw std::out_of_range(
            "Graph delete_edge failed, edge "
            "[" + source + "] to [" + destination + "]"
            " doesn't exist."
    );
  }

  Node::Restricted token;
  nodes_[source]->outbound_edges(token).erase(destination);
  nodes_[destination]->inbound_edges(token).erase(source);
}


void Graph::delete_node(const std::string & id)
{
  Node::Restricted token;
  auto itr = nodes_.find(id);

  // Check if node exists
  if (itr == nodes_.end()) {
    throw std::out_of_range(
            "Graph delete_node failed, node [" + id + "] doesn't exist."
    );
  }

  // Erase inbound edges
  for (auto & inbound_edge : itr->second->inbound_edges()) {
    nodes_[inbound_edge.first]->outbound_edges(token).erase(id);
  }

  // Erase outbound edges
  for (auto & outbound_edge : itr->second->outbound_edges()) {
    nodes_[outbound_edge.first]->inbound_edges(token).erase(id);
  }

  nodes_.erase(itr);
}

void Graph::prune()
{
  auto itr = nodes_.begin();
  while (itr != nodes_.end()) {
    if (
      itr->second->inbound_edges().empty() &&
      itr->second->outbound_edges().empty())
    {
      itr = nodes_.erase(itr);
    } else {
      itr++;
    }
  }
}

bool Graph::has_node(const std::string & id) const
{
  return nodes_.find(id) != nodes_.end();
}

Node::ConstPtr Graph::get_node(const std::string & id) const
{
  auto itr = nodes_.find(id);

  // Check if node exists
  if (itr == nodes_.end()) {
    throw std::out_of_range(
            "Graph get_node failed, node [" + id + "] doesn't exist."
    );
  }
  return itr->second;
}

std::unordered_map<std::string, Node::ConstPtr>
Graph::get_all_nodes() const
{
  return {nodes_.begin(), nodes_.end()};
}

std::map<std::string, Node::ConstPtr>
Graph::get_all_nodes_ordered() const
{
  return {nodes_.begin(), nodes_.end()};
}

bool Graph::empty() const
{
  return nodes_.empty();
}

void Graph::dump(std::ostream & oss) const
{
  std::vector<std::string> edges;
  oss << "digraph DAG {\n";

  for_each_node_ordered(
    [&oss](Node::Ptr node) {
      std::map<std::string, Edge> ordered_outbound_edges{
        node->outbound_edges().begin(),
        node->outbound_edges().end(),
      };
      for (auto & outbound_edge : ordered_outbound_edges) {
        oss << "  \"" << node->id() << "\" -> \"";
        oss << outbound_edge.first;
        oss << "\" [label=" << outbound_edge.second.type << "];" << std::endl;
      }
    });
  oss << "}" << std::endl;
}

}  // namespace data

}  // namespace rmf2_scheduler
