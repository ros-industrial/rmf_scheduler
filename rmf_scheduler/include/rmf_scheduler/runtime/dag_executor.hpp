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

#ifndef RMF_SCHEDULER__RUNTIME__DAG_EXECUTOR_HPP_
#define RMF_SCHEDULER__RUNTIME__DAG_EXECUTOR_HPP_

#include <memory>
#include <functional>
#include <future>
#include <string>
#include "rmf_scheduler/data/dag.hpp"

namespace tf
{
template<typename T>
class Future;
class Executor;
}  // namespace tf

namespace rmf_scheduler
{

namespace runtime
{

/// Executor that executes tasks in order based on the DAG.
/// Unless specificed otherwise, all methods in this class are thread-safe.
class DAGExecutor
{
public:
  using Work = std::function<void ()>;
  using WorkGenerator = std::function<Work(const std::string &)>;
  static Work EmptyWork();

  explicit DAGExecutor(unsigned int concurrency = std::thread::hardware_concurrency());

  virtual ~DAGExecutor();

  std::shared_future<void> run(const data::DAG::Description &, WorkGenerator);

  void cancel_and_next(const data::DAG::Description &, WorkGenerator);

  void cancel();

  bool ongoing() const;

protected:
  void _cancel_and_next();
  std::unique_ptr<tf::Executor> executor_;
  std::unique_ptr<data::DAG> dag_;
  std::unique_ptr<data::DAG> next_dag_;
  mutable std::mutex next_dag_mtx_;
  std::shared_ptr<tf::Future<void>> cancel_handler_;
  std::shared_future<void> future_;
  std::shared_ptr<std::future<void>> cancel_future_;
};

}  // namespace runtime

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__RUNTIME__DAG_EXECUTOR_HPP_
