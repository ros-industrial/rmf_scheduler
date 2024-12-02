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

#ifndef RMF2_SCHEDULER__UTILS__TREE_CONVERTER_HPP_
#define RMF2_SCHEDULER__UTILS__TREE_CONVERTER_HPP_

#include <memory>
#include <ostream>
#include <string>
#include <unordered_map>

#include "rmf2_scheduler/data/node.hpp"
#include "rmf2_scheduler/data/graph.hpp"

namespace rmf2_scheduler
{

namespace utils
{

class TreeConversion
{
public:
  class Implementation;

  TreeConversion();

  ~TreeConversion();

  /// convert_to_tree
  /**
   * main function that converts a DAG to BT
   * \throws std::logic error if graph contains unconnected nodes
   * \throws std::logic_error if cycles are detected in graph during conversion
   * \return std::string containing a behaviour tree in XML format
   */
  [[nodiscard]] std::string convert_to_tree(const data::Graph & graph);

  /// function to check whether a graph contains cycle(s)
  bool is_cyclic(const data::Graph & graph) const;

private:
  /// make_tree
  /**
   * main point of entry for the recursive function to generate trees
   */
  void make_tree(const data::Graph & graph, std::ostream & oss);

  /// helper function to check whether a graph contains cycle(s)
  bool _is_cyclic_util(
    const std::string & id,
    std::unordered_map<std::string, bool> & visited,
    std::unordered_map<std::string, bool> & rec_stack,
    std::unordered_map<std::string, data::Node::ConstPtr> node_list) const;

  std::unique_ptr<Implementation> p_;
};
}  // namespace utils

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__UTILS__TREE_CONVERTER_HPP_
