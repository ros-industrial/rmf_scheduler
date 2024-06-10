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

#ifndef RMF_SCHEDULER__DATA__NODE_HPP_
#define RMF_SCHEDULER__DATA__NODE_HPP_

#include <memory>
#include <string>
#include <unordered_set>

namespace rmf_scheduler
{

namespace data
{

class Node
{
public:
  using Ptr = std::shared_ptr<Node>;
  using ConstPtr = std::shared_ptr<const Node>;
  using UPtr = std::unique_ptr<Node>;
  using ConstUPtr = std::unique_ptr<const Node>;

  class Restricted;

  /// Constructor to create a node with an ID
  explicit Node(const std::string & id);

  /// Destructor
  virtual ~Node();

  // Not copiable
  Node(const Node &) = delete;
  Node & operator=(const Node &) = delete;

  // Not movable
  Node(Node &&) = delete;
  Node & operator=(Node &&) = delete;

  /// Node ID
  const std::string & id() const;

  /// Retrieve inbound edges for the node
  const std::unordered_set<std::string> & inbound_edges() const;

  /// Retrieve outbound edges for the node
  const std::unordered_set<std::string> & outbound_edges() const;

  /// Restricted access to non-const inbound edges
  std::unordered_set<std::string> & inbound_edges(const Restricted &);

  /// Restricted access to non-const outbound edges
  std::unordered_set<std::string> & outbound_edges(const Restricted &);

private:
  std::string id_;
  std::unordered_set<std::string> inbound_edges_;
  std::unordered_set<std::string> outbound_edges_;
};

/// Private class to allow restricted access to modify node information
class Node::Restricted
{
private:
  Restricted();
  virtual ~Restricted();
  friend class Graph;
};

}  // namespace data
}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__DATA__NODE_HPP_
