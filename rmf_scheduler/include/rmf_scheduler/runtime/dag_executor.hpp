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

  explicit DAGExecutor(unsigned int concurrency = std::thread::hardware_concurrency());
  DAGExecutor(const DAGExecutor &) = delete;
  DAGExecutor & operator=(const DAGExecutor &) = delete;
  DAGExecutor(DAGExecutor &&);
  DAGExecutor & operator=(DAGExecutor && rhs);

  virtual ~DAGExecutor();

  std::shared_future<void> run(data::DAG &, WorkGenerator);

  void cancel();

protected:
  std::unique_ptr<tf::Executor> executor_;
  std::shared_ptr<tf::Future<void>> future_;
};

}  // namespace runtime

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__RUNTIME__DAG_EXECUTOR_HPP_
