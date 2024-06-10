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

#include "rmf_scheduler/data/node.hpp"

namespace rmf_scheduler
{

namespace data
{

Node::Node(const std::string & id)
: id_(id)
{
}

Node::~Node()
{
}

const std::string & Node::id() const
{
  return id_;
}

const std::unordered_set<std::string> & Node::inbound_edges() const
{
  return inbound_edges_;
}

const std::unordered_set<std::string> & Node::outbound_edges() const
{
  return outbound_edges_;
}

std::unordered_set<std::string> & Node::inbound_edges(
  const Node::Restricted &
)
{
  return inbound_edges_;
}

std::unordered_set<std::string> & Node::outbound_edges(
  const Node::Restricted &
)
{
  return outbound_edges_;
}

Node::Restricted::Restricted()
{
}

Node::Restricted::~Restricted()
{
}

}  // namespace data

}  // namespace rmf_scheduler
