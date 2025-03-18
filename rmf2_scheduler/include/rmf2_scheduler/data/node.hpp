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

#ifndef RMF2_SCHEDULER__DATA__NODE_HPP_
#define RMF2_SCHEDULER__DATA__NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rmf2_scheduler/data/edge.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace data
{

class Node
{
public:
  RS_SMART_PTR_DEFINITIONS(Node)

  class Restricted;

  /// Constructor to create a node with an ID
  explicit Node(const std::string & id);

  /// Destructor
  virtual ~Node();

  /// Node ID
  const std::string & id() const;

  /// Retrieve inbound edges for the node
  const std::unordered_map<std::string, Edge> & inbound_edges() const;

  /// Retrieve outbound edges for the node
  const std::unordered_map<std::string, Edge> & outbound_edges() const;

  /// Restricted access to non-const inbound edges
  std::unordered_map<std::string, Edge> & inbound_edges(const Restricted &);

  /// Restricted access to non-const outbound edges
  std::unordered_map<std::string, Edge> & outbound_edges(const Restricted &);

  inline bool
  operator==(const Node & rhs) const
  {
    if (this->id_ != rhs.id_) {return false;}
    if (this->inbound_edges_ != rhs.inbound_edges_) {return false;}
    if (this->outbound_edges_ != rhs.outbound_edges_) {return false;}
    return true;
  }

  bool
  operator!=(const Node & rhs) const
  {
    return !(*this == rhs);
  }

private:
  std::string id_;
  std::unordered_map<std::string, Edge> inbound_edges_;
  std::unordered_map<std::string, Edge> outbound_edges_;
};

/// Private class to allow restricted access to modify node information
class Node::Restricted
{
private:
  Restricted();
  virtual ~Restricted();
  friend class Graph;
  friend class TestNode;
};

}  // namespace data
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__DATA__NODE_HPP_
