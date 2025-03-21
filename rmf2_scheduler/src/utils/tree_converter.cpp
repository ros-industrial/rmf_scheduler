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

#include <tinyxml2.h>

#include <algorithm>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "rmf2_scheduler/utils/dag_helper.hpp"
#include "rmf2_scheduler/utils/tree_converter.hpp"

namespace rmf2_scheduler
{

namespace utils
{

class TreeConversion::Implementation
{
public:
  /// Recursive Depth First Search to get all ancestor nodes
  void DFS(
    const std::string & id, std::unordered_map<std::string, size_t> & ancestors,
    std::unordered_map<std::string, bool> visited,
    const std::unordered_map<std::string, data::Node::ConstPtr> & node_map,
    int distance)
  {
    visited[id] = true;
    for (const auto & [inbound_node_id, edge] : node_map.at(id)->inbound_edges()) {
      if (!visited.at(inbound_node_id)) {
        ancestors[inbound_node_id] = distance + 1;
        DFS(inbound_node_id, ancestors, visited, node_map, distance + 1);
      }
    }
  }

  /// helper function to get all ancestors of a node
  std::unordered_map<std::string, size_t> get_parent_nodes(
    const std::string & node_id, const std::unordered_map<std::string,
    data::Node::ConstPtr> & node_map)
  {
    std::unordered_map<std::string, size_t> ancestors;
    std::unordered_map<std::string, bool> visited;
    for (const auto & [node_id, node] : node_map) {
      visited[node_id] = false;
    }
    DFS(node_map.at(node_id)->id(), ancestors, visited, node_map, 0);
    return ancestors;
  }

  /// helper function to find common ancestor to get out of recursive sequence/parallel tags
  /// Naive implementation, can be improved: https://www3.cs.stonybrook.edu/~bender/pub/JALG05-daglca.pdf
  std::string find_common_ancestor(
    std::vector<std::string> & inbound_nodes,
    const std::unordered_map<std::string, data::Node::ConstPtr> & node_map)
  {
    std::unordered_map<std::string, size_t> common_ancestor_map;
    std::unordered_map<std::string, size_t> common_ancestor_distance_map;
    std::multimap<size_t, std::string> common_ancestors;
    std::vector<std::unordered_map<std::string, size_t>> inbound_nodes_ancestors;
    // finding ancestors of all inbound nodes
    for (const auto & node : inbound_nodes) {
      inbound_nodes_ancestors.push_back(get_parent_nodes(node, node_map));
    }
    auto count = inbound_nodes.size();
    for (const auto & inbound_node_ancestors : inbound_nodes_ancestors) {
      for (const auto & [ancestor, distance] : inbound_node_ancestors) {
        common_ancestor_map[ancestor] += 1;
        auto itr = common_ancestor_distance_map.find(ancestor);
        if (itr == common_ancestor_distance_map.end()) {
          common_ancestor_distance_map[ancestor] = distance;
        } else if (itr->second > distance) {
          itr->second = distance;
        }
      }
    }
    // if the count of the ancestor equals to the number of inbound nodes,
    // it is a common ancestor
    for (const auto & [node_id, common_count] : common_ancestor_map) {
      if (common_count == count) {
        common_ancestors.emplace(common_ancestor_distance_map[node_id], node_id);
      }
    }

    if (!common_ancestors.empty()) {
      return common_ancestors.begin()->second;
    } else {
      return "";
    }
  }

  /// recursively_create_tree
  /**
   * \param visited tracks visited nodes in the recursively call
   * \param node_map map containing list of nodes in the DAG
   * \param xml_nodes map pairing tinyxml2 node with DAG node for node level tracking
   * \param wait_nodes nodes waiting for dependencies to be cleared
   * \param parent tinyxml2 tag the current node is placed under
   * \param doc tinyxml2 doc
   */
  void recursively_create_tree(
    data::Node::ConstPtr node, std::unordered_map<std::string, bool> & visited,
    const std::unordered_map<std::string, data::Node::ConstPtr> & node_map,
    std::unordered_map<std::string, tinyxml2::XMLNode *> & xml_nodes,
    std::set<std::string> & wait_nodes, tinyxml2::XMLNode * parent,
    tinyxml2::XMLDocument & doc,
    const MakeTreeCallback & callback
  )
  {
    visited.at(node->id()) = true;
    for (const auto & [id, edge] : node->inbound_edges()) {
      if (!visited.at(id)) {
        wait_nodes.insert(node->id());
        return;
      }
    }
    // Wait nodes are nodes that need to be cleared once all dependencies are cleared.
    // Finding common ancestor to get out of recursive sequence/parallel tags
    if (wait_nodes.find(node->id()) != wait_nodes.end()) {
      std::vector<std::string> immediate_inbound_nodes;
      for (const auto & [id, edge] : node->inbound_edges()) {
        immediate_inbound_nodes.push_back(id);
      }
      auto common_ancestor = find_common_ancestor(
        immediate_inbound_nodes, node_map);
      // There maybe cases where there are no common ancestors
      // if so, parent will be one level before
      if (common_ancestor.empty()) {
        parent = parent->Parent();
      } else {
        parent = xml_nodes.at(common_ancestor);
      }
      wait_nodes.erase(node->id());
    }
    // if there are any errors in the tree creation structure, most likely a logic error
    // in the following block of code. Adopted from: https://arxiv.org/pdf/2101.01964.pdf
    if (node->outbound_edges().size() == 0) {
      tinyxml2::XMLElement * leaf_node = doc.NewElement(
        callback(node->id()).c_str()
      );
      parent->InsertEndChild(leaf_node);
      xml_nodes.insert({node->id(), parent});
    } else if (node->outbound_edges().size() == 1) {
      tinyxml2::XMLNode * tag = doc.NewElement("Sequence");
      tinyxml2::XMLElement * task = doc.NewElement(
        callback(node->id()).c_str()
      );
      parent->InsertEndChild(tag);
      tag->InsertEndChild(task);
      xml_nodes.insert({node->id(), parent});
      recursively_create_tree(
        node_map.at(node->outbound_edges().begin()->first), visited,
        node_map, xml_nodes, wait_nodes, parent, doc, callback);
    } else {
      tinyxml2::XMLNode * tag = doc.NewElement("Sequence");
      parent->InsertEndChild(tag);
      tinyxml2::XMLElement * task = doc.NewElement(
        callback(node->id()).c_str()
      );
      tag->InsertEndChild(task);
      xml_nodes.insert({node->id(), tag});
      tinyxml2::XMLNode * sub_tag = doc.NewElement("Parallel");
      tag->InsertEndChild(sub_tag);
      for (const auto & [id, edge] : node->outbound_edges()) {
        tinyxml2::XMLNode * branch_seq = doc.NewElement("Sequence");
        sub_tag->InsertEndChild(branch_seq);
        recursively_create_tree(
          node_map.at(id), visited, node_map, xml_nodes, wait_nodes, branch_seq,
          doc, callback
        );
      }
    }
  }

  /// Recursive function to remove all Control tags with no children
  void validate_tree(
    tinyxml2::XMLNode * node, tinyxml2::XMLDocument & doc,
    const std::unordered_map<std::string, data::Node::ConstPtr> & node_map)
  {
    for (; node; node = node->NextSibling()) {
      // Check if <Sequence> or <Parallel> tags need to be removed.
      // Separate from recursive call as not all tags are first child of nodes
      if (!strcmp(node->Value(), "Parallel") || !strcmp(node->Value(), "Sequence")) {
        // tags with single nodes are ones we want to remove
        if (node->FirstChild() == node->LastChild()) {
          auto * parent = node->Parent();
          // this prevents accidentally accessing nodes with no children
          if (!node->NoChildren()) {
            auto * child = node->FirstChild()->DeepClone(node->GetDocument());
            parent->InsertAfterChild(node, child);
          }
          parent->DeleteChild(node);
          // looping from the main sequence in the case of deeply nested single child tags
          node = doc.FirstChild()->FirstChild()->FirstChild();
        }
      }
      if (node->FirstChild()) {
        validate_tree(node->FirstChild(), doc, node_map);
      }
    }
  }
};

TreeConversion::TreeConversion()
: p_(std::make_unique<Implementation>()) {}

TreeConversion::~TreeConversion() {}

std::string TreeConversion::convert_to_tree(
  const data::Graph & graph,
  const MakeTreeCallback & callback
)
{
  return convert_to_tree("dag-numbers", graph, callback);
}

std::string TreeConversion::convert_to_tree(
  const std::string & id,
  const data::Graph & graph,
  const MakeTreeCallback & callback
)
{
  if (graph.empty()) {
    return "";
  }
  // throws only if there is more than 1 node
  // in the case of the dag only containing one task
  if (graph.get_all_nodes().size() > 1) {
    graph.for_each_node(
      [](data::Node::ConstPtr node) {
        if (node->inbound_edges().empty() && node->outbound_edges().empty()) {
          throw std::logic_error(
            "Graph contains unconnected node(s)! Cannot convert graph to tree!");
        }
      });
  }
  if (!is_valid_dag(graph)) {
    throw std::logic_error("Cycle(s) detected in graph! Cannot convert graph to tree!");
  }
  std::ostringstream oss;
  make_tree(id, graph, oss, callback);
  return oss.str();
}

void TreeConversion::make_tree(
  const std::string & id,
  const data::Graph & graph,
  std::ostream & oss,
  const MakeTreeCallback & callback
)
{
  std::unordered_map<std::string, bool> visited;

  // initialising default tags for behaviour trees
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLElement * root = doc.NewElement("root");
  root->SetAttribute("BTCPP_format", "4");
  doc.InsertEndChild(root);
  tinyxml2::XMLElement * bt_id = doc.NewElement("BehaviorTree");
  bt_id->SetAttribute("ID", id.c_str());
  root->InsertEndChild(bt_id);
  tinyxml2::XMLNode * main_seq = doc.NewElement("Sequence");
  bt_id->InsertEndChild(main_seq);

  std::vector<data::Node::ConstPtr> entry_nodes;
  // used for tracking node levels in recursive call
  std::unordered_map<std::string, tinyxml2::XMLNode *> xml_nodes;
  // used to deal with nodes with dependencies
  std::set<std::string> wait_nodes;

  auto node_map = graph.get_all_nodes();
  graph.for_each_node(
    [&entry_nodes, &visited](data::Node::ConstPtr node) {
      if (node->inbound_edges().empty()) {
        // entry nodes will be the main points of entry for recursion
        entry_nodes.emplace_back(node);
      }
      visited[node->id()] = false;
    });

  if (entry_nodes.size() > 1) {
    tinyxml2::XMLNode * tag = doc.NewElement("Parallel");
    main_seq->InsertEndChild(tag);
    for (const auto & node : entry_nodes) {
      p_->recursively_create_tree(
        node, visited, node_map, xml_nodes, wait_nodes, tag, doc, callback
      );
    }
  } else {
    p_->recursively_create_tree(
      entry_nodes.front(), visited, node_map, xml_nodes, wait_nodes, main_seq, doc, callback
    );
  }
  p_->validate_tree(main_seq, doc, node_map);
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  auto str = printer.CStr();
  oss << str;
}

}  // namespace utils

}  // namespace rmf2_scheduler
