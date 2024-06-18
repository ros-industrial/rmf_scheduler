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

#include <sstream>
#include <stack>
#include <algorithm>
#include "rmf_scheduler/utils/dag_bt_converter.hpp"

namespace rmf_scheduler
{
namespace bt_converter
{

BTConverter::BTConverter(const std::string & dag_bt_str)
: ordered_tasks_{dag_bt_str} {}

BTConverter::~BTConverter()
{
}

std::string BTConverter::get_xml_str() const
{
  return xml_;
}

std::string BTConverter::_extractTask(const std::string & task)
{
  return task.substr(task.find(" ") + 1);
}

size_t BTConverter::_countParallelLines(const std::string & obj)
{
  auto parallelStart = obj.find("/");
  auto parallelEnd = obj.find_last_of("/");
  size_t count{0};
  count =
    (parallelStart == std::string::npos) ? 0 : (parallelStart ==
    parallelEnd) ? parallelEnd + 1 : (parallelEnd - parallelStart + 1);
  return count;
}

std::unordered_map<std::string, std::string> BTConverter::_mapValues(std::string task_info)
{
  std::unordered_map<std::string, std::string> task_map_info;
  // remove trailing spaces from task
  task_info.erase(std::remove(task_info.begin(), task_info.end(), ' '), task_info.end());
  std::istringstream iss{task_info};
  std::string s;
  // separating key-value pair of the task info
  while (std::getline(iss, s, ',')) {
    task_map_info[s.substr(0, s.find(":"))] = s.substr(s.find(":") + 1);
  }
  return task_map_info;
}

std::vector<std::vector<std::string>> BTConverter::_get_parsed_ordered_tasks()
{
  size_t pll_pos, seq_pos, leaf_pos;
  std::vector<std::vector<std::string>> ordered_task_list, leaf_node_list;
  std::vector<std::string> subtree_list;
  bool in_subtree{false}, new_branch{false};
  seq_pos = 0;
  pll_pos = ordered_tasks_.find("/", seq_pos);
  leaf_pos = ordered_tasks_.find(",", seq_pos);
  while ((seq_pos = ordered_tasks_.find("->", seq_pos)) < std::string::npos) {
    std::string tmp;
    std::vector<std::string> tmp_vec;
    size_t pll_lines_count{0};
    if (pll_pos != std::string::npos) {
      // if parallel separator trails sequence arrow, following task is a parallel node
      if (pll_pos < seq_pos) {
        // extracting the index of the last parallel line
        auto tmp_pll = ordered_tasks_.find_first_not_of("/", pll_pos);
        if (pll_pos == (tmp_pll - 1)) {  // if it's only one parallel line, it's a subtree
          if (new_branch) {
            ordered_task_list.push_back(subtree_list);
          } else {
            new_branch = true;
            in_subtree = true;
          }
          subtree_list.clear();
        }
        pll_lines_count = tmp_pll - pll_pos;
        // if there are parallel lines, adds it into the tmp string buffer
        tmp = ordered_tasks_.substr(pll_pos, pll_lines_count);
        // go to the next instance of a parallel node
        pll_pos = ordered_tasks_.find("/", tmp_pll);
      }
    }
    // adds the task to the string buffer
    tmp += ordered_tasks_.substr(seq_pos, (ordered_tasks_.find(" ", seq_pos + 3) - seq_pos));
    // if comma separator trails sequence arrow, following task is a leaf node
    if (leaf_pos < seq_pos) {
      if (in_subtree) {
        tmp_vec.push_back(tmp);
        leaf_node_list.push_back(tmp_vec);
      }
      leaf_pos = ordered_tasks_.find(",", seq_pos);  // find next instance of comma separator
    } else {
      subtree_list.push_back(tmp);
    }
    if (!in_subtree) {
      tmp_vec.push_back(tmp);
      ordered_task_list.push_back(tmp_vec);
    }
    seq_pos += (tmp.size() - pll_lines_count);  // exclude parallel separators
    if ((ordered_tasks_.find("->", seq_pos)) >= std::string::npos) {
      if (in_subtree) {
        ordered_task_list.push_back(subtree_list);
      }
    }
  }
  for (auto & leaf : leaf_node_list) {
    ordered_task_list.push_back(leaf);
  }
  return ordered_task_list;
}

// Function to generate XML from input string representing task dependencies
void BTConverter::generate_tree(const std::string & bt_id, const TaskInfo & task)
{
  auto input = _get_parsed_ordered_tasks();
  bool inSequence = false;   // flag to track if currently in a sequence block
  // flag to check if the previous object was a parallel object
  std::pair<size_t, bool> isPrevInSequence(0, false);
  size_t parallelStart;   // variable to store the first occurence of a parallel line in a string
  size_t parallelCount;   // variable to store the number of parallel lines in a string
  std::string movementorder;
  bool isLastInstance = false;   // flag to track last instance of a sequence
  int count{1};
  // Map to keep track of all parallel lines in the tree
  std::unordered_map<size_t, int> parallelCountMap;
  // Tracker for parallel tag lines
  std::unordered_map<size_t, bool> isParallelFirstInstance;
  std::stack<tinyxml2::XMLNode *> nodeStack;  // Tracker for indentation level of XML

  // tinyxml2 XML header generation
  tinyxml2::XMLElement * root = doc_.NewElement("root");
  root->SetAttribute("BTCPP_format", "4");
  doc_.InsertEndChild(root);
  tinyxml2::XMLElement * btId = doc_.NewElement("BehaviorTree");
  btId->SetAttribute("ID", bt_id.c_str());
  root->InsertEndChild(btId);
  tinyxml2::XMLNode * mainSeq = doc_.NewElement("Sequence");
  btId->InsertEndChild(mainSeq);
  nodeStack.push(mainSeq);


  // counting all instances of all parallel lines in the tree
  std::for_each(
    input.begin(), input.end(), [&](std::vector<std::string> subtree) {
      for (auto & obj : subtree) {
        if ((parallelStart = obj.find("/")) != std::string::npos) {
          // variable set as key for tracking of parallel lines
          parallelCount = _countParallelLines(obj);
          parallelCountMap[parallelCount] += 1;
          isParallelFirstInstance[parallelCount] = true;
        }
      }
    });
  // Iterate through vector of vectors
  for (auto subtree = input.begin(); subtree != input.end(); ++subtree) {
    isPrevInSequence = {0, false};
    // single out last element for closing tags
    auto last_element = subtree->back();
    isLastInstance = false;
    // Iterate through vector of strings
    for (auto obj = subtree->begin(); obj != subtree->end(); ++obj) {
      if (*obj == last_element) {
        isLastInstance = true;
      }
      parallelStart = obj->find("/");
      // check if the current task is parallel to another
      bool isParallelObj = (parallelStart != std::string::npos);
      // if the node is parallel
      if (isParallelObj) {
        parallelCount = _countParallelLines(*obj);
        // if it's first instance of parallel lines, generate opening tag
        if (isParallelFirstInstance[parallelCount]) {
          tinyxml2::XMLNode * subParallel = doc_.NewElement("Parallel");
          nodeStack.top()->InsertEndChild(subParallel);
          nodeStack.push(subParallel);
          isParallelFirstInstance[parallelCount] = false;
        }
        parallelCountMap[parallelCount] -= 1;
        // if vector contains one element and it is parallel, no sequence needed
        if (subtree->size() != 1) {
          // out of sequence, remove pointer to sequence tag from the stack
          if (isPrevInSequence.first >= parallelCount && isPrevInSequence.second) {
            nodeStack.pop();
          }
          // if next element of the subtree is not the last element
          if (std::next(obj) != subtree->end()) {
            // start sequence if next element isn't same parallelity
            inSequence = parallelCount > _countParallelLines(*(std::next(obj)));
          }
          if (*obj == last_element) {
            inSequence = false;
          }
          if (inSequence) {
            tinyxml2::XMLElement * subSeq = doc_.NewElement("Sequence");
            nodeStack.top()->InsertEndChild(subSeq);
            nodeStack.push(subSeq);
            isPrevInSequence = {parallelCount, true};
          } else {
            isPrevInSequence = {parallelCount, false};
          }
        }
      } else {
        if (subtree->size() == 1) {
          while (nodeStack.size() != 1) {
            nodeStack.pop();
          }
        }
      }
      // XML script generation
      tinyxml2::XMLElement * script = doc_.NewElement("Script");
      // storing returning task info string from function callback
      std::string task_info = task(_extractTask(*obj));
      // remove trailing spaces from task
      task_info.erase(std::remove(task_info.begin(), task_info.end(), ' '), task_info.end());
      std::istringstream iss{task_info};
      std::unordered_map<std::string, std::string> task_map_info;
      std::string s;
      // separating key-value pair of the task info
      while (std::getline(iss, s, ',')) {
        task_map_info[s.substr(0, s.find(":"))] = s.substr(s.find(":") + 1);
      }
      // concatenating movement order script
      std::string movement_order = "MO" + std::to_string(count) + ":='" + task_map_info["TrayID"] +
        "," + _extractTask(*obj) + "," + task_map_info["OriginID"] + "," +
        task_map_info["DestinationID"] + "'";
      script->SetAttribute("code", movement_order.c_str());
      tinyxml2::XMLElement * node = doc_.NewElement("MovementOrder");
      std::string mo_attr = "{MO" + std::to_string(count) + "}";
      node->SetAttribute("MO", mo_attr.c_str());
      nodeStack.top()->InsertEndChild(script);
      nodeStack.top()->InsertEndChild(node);
      ++count;
      if (subtree->size() == 1) {
        break;
      }
      // if it's last instance of parallel lines per subtree, generate closing tags
      if (isLastInstance) {
        for (size_t i = nodeStack.size(); i != 2; --i, nodeStack.pop()) {
        }
        if (parallelCountMap[1] == 0) {
          auto node = nodeStack.top();
          nodeStack.pop();
          nodeStack.top()->InsertEndChild(node);
        }
      }
    }
  }
  tinyxml2::XMLPrinter printer;
  doc_.Print(&printer);
  xml_ = printer.CStr();
}
}  // namespace bt_converter
}  // namespace rmf_scheduler
