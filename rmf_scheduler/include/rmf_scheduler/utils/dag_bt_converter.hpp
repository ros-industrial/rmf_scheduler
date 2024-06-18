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
#include <string>
#include <unordered_map>
#include <vector>
#include <functional>

#ifndef RMF_SCHEDULER__UTILS__DAG_BT_CONVERTER_HPP_
#define RMF_SCHEDULER__UTILS__DAG_BT_CONVERTER_HPP_

namespace rmf_scheduler
{
namespace bt_converter
{
class BTConverter
{
public:
  using TaskInfo = std::function<std::string(const std::string &)>;
  BTConverter() = delete;
  // can only instantiate class if formatted dag std::string is given in constructor
  explicit BTConverter(const std::string & dag_bt_str);
  BTConverter(const BTConverter &) = delete;
  BTConverter & operator=(const BTConverter &) = delete;
  virtual ~BTConverter();
  // returns a string object of a behavior tree in xml form
  std::string get_xml_str() const;
  // takes in a std::function and a string obj representing the ID for the BT xml generation
  void generate_tree(const std::string & bt_id, const TaskInfo & id);

private:
  std::string _extractTask(const std::string & task);
  size_t _countParallelLines(const std::string & obj);
  std::unordered_map<std::string, std::string> _mapValues(std::string task_info);
  std::vector<std::vector<std::string>> _get_parsed_ordered_tasks();
  std::string xml_;
  std::string ordered_tasks_;
  tinyxml2::XMLDocument doc_;
};
}  // namespace bt_converter
}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__UTILS__DAG_BT_CONVERTER_HPP_
