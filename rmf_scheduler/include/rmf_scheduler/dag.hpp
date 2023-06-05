// Copyright 2023 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF_SCHEDULER__DAG_HPP_
#define RMF_SCHEDULER__DAG_HPP_

#include <cstdarg>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rmf_scheduler/exception.hpp"

namespace tf
{
class Task;
class Taskflow;
}  // namespace tf

namespace rmf_scheduler
{

class DAG
{
  friend class DAGExecutor;

public:
  using DependencyInfo = std::vector<std::string>;
  using Description = std::unordered_map<std::string, DependencyInfo>;
  using NodeList = std::unordered_map<std::string, std::unique_ptr<tf::Task>>;

  DAG();
  DAG(const DAG &) = delete;
  DAG & operator=(const DAG &) = delete;
  DAG(DAG &&);
  DAG & operator=(DAG && rhs);

  /// Construct a DAG based on description
  /**
   * \param[in] description
   * \param[in] safe check if the graph remain cyclic
   * \throw DAGCyclicException
   */
  explicit DAG(const Description &, bool safe = false);

  virtual ~DAG();

  void add_node(
    const std::string & id);

  void delete_node(
    const std::string & id);

  /// Add dependency
  /**
   * \param[in] id
   * \param[in] dependency_info
   * \param[in] safe check if the graph remain cyclic
   * \throw DAGCyclicException
   */
  void add_dependency(
    const std::string & id,
    const DependencyInfo & dependency_info,
    bool safe = false);

  void clear_dependency(
    const std::string & id);

  bool has_node(const std::string &) const;

  /// Check if the graph contains a cycle with the new dependency addition
  /**
   * Detail explanation can be found in
   * <https://www.geeksforgeeks.org/detect-cycle-in-a-graph/>
   *
   * \return true if the graph contains a cycle, else false.
   */
  bool is_cyclic() const;

  std::vector<std::string> entry_nodes() const;

  std::vector<std::string> end_nodes() const;

  std::vector<std::string> all_nodes() const;

  std::string dot() const;

  Description description() const;

  DependencyInfo get_dependency_info(const std::string &) const;

protected:
  std::unique_ptr<tf::Taskflow> taskflow_;
  NodeList node_list_;

private:
  // DFS function to find if a cycle exists
  bool _is_cyclic_until(
    const std::string & id,
    std::unordered_map<std::string, bool> & visited,
    std::unordered_map<std::string, bool> & rec_stack) const;

  Description temp_description_info_;
};


class DAGIDException : public IDException
{
public:
  template<typename ... Args>
  DAGIDException(const char * id, const char * msg, Args && ... args)
  : IDException(id, msg, std::forward<Args>(args) ...)
  {
    add_prefix("DAGIDException:\n  ");
  }
};


class DAGCyclicException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  DAGCyclicException(const char * msg, Args && ... args)
  : ExceptionTemplate(msg, std::forward<Args>(args) ...)
  {
    add_prefix("DAGCyclicException:\n  ");
  }
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__DAG_HPP_
