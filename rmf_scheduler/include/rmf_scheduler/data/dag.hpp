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

#ifndef RMF_SCHEDULER__DATA__DAG_HPP_
#define RMF_SCHEDULER__DATA__DAG_HPP_

#include <cstdarg>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <utility>
#include <vector>

#include "rmf_scheduler/data/graph.hpp"
#include "rmf_scheduler/exception.hpp"
#include "rmf_scheduler/error_code.hpp"

namespace rmf_scheduler
{

namespace runtime
{
class DAGExecutor;
}  // namespace runtime

namespace data
{
class DAG
{
  friend class runtime::DAGExecutor;

public:
  using DependencyInfo = std::vector<std::string>;
  using Description = std::unordered_map<std::string, DependencyInfo>;
  using TaskInfo = std::function<std::string(const std::string &)>;

  DAG();

  // Not copiable
  DAG(const DAG &) = delete;
  DAG & operator=(const DAG &) = delete;

  // Movable
  DAG(DAG &&) = default;
  DAG & operator=(DAG &&) = default;

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

  void update_node(
    const std::string & id,
    const std::string & new_id);

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

  std::unordered_set<std::string> entry_nodes() const;

  std::unordered_set<std::string> end_nodes() const;

  std::vector<std::string> all_nodes() const;

  std::string dot() const;

  Description description() const;

  DependencyInfo get_dependency_info(const std::string &) const;

  std::string generate_bt_xml(const std::string & bt_id, const TaskInfo & API_callback);
  // not a const function due to adding of null node if >1 entry node
  std::string generate_ordered_tree();

protected:
  Graph graph_;

private:
  // DFS function to find if a cycle exists
  bool _is_cyclic_until(
    const std::string & id,
    std::unordered_map<std::string, bool> & visited,
    std::unordered_map<std::string, bool> & rec_stack) const;

  void _get_tree(
    const std::string & id, std::unordered_map<std::string, bool> & visited,
    bool & if_task_before, bool & is_parallel, std::ostream & oss, size_t & parallel_count,
    std::stack<size_t> & parallel_count_stack);

  Description temp_description_info_;
};

}  // namespace data

namespace exception
{

class DAGIDException : public IDException
{
public:
  template<typename ... Args>
  DAGIDException(const char * id, const char * msg, Args && ... args)
  : IDException(
      ErrorCode::FAILURE | ErrorCode::INVALID_ID | ErrorCode::INVALID_DEPENDENCY,
      id, msg, std::forward<Args>(args) ...)
  {
    add_prefix("DAGIDException:\n  ");
  }
};


class DAGCyclicException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  DAGCyclicException(const char * msg, Args && ... args)
  : ExceptionTemplate(
      ErrorCode::FAILURE | ErrorCode::INVALID_LOGIC | ErrorCode::INVALID_DEPENDENCY,
      msg, std::forward<Args>(args) ...)
  {
    add_prefix("DAGCyclicException:\n  ");
  }
};

}  // namespace exception

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__DATA__DAG_HPP_
