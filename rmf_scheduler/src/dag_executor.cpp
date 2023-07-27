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

#include <utility>

#include "rmf_scheduler/dag_executor.hpp"
#include "taskflow/taskflow.hpp"

namespace rmf_scheduler
{

DAGExecutor::DAGExecutor(unsigned int concurrency)
: executor_(std::make_unique<tf::Executor>(concurrency))
{
}

DAGExecutor::DAGExecutor(DAGExecutor && rhs)
{
  executor_ = std::move(rhs.executor_);
  future_ = std::move(rhs.future_);
}

DAGExecutor & DAGExecutor::operator=(DAGExecutor && rhs)
{
  executor_ = std::move(rhs.executor_);
  future_ = std::move(rhs.future_);
  return *this;
}

DAGExecutor::~DAGExecutor()
{
}

std::shared_future<void> DAGExecutor::run(
  DAG & dag,
  WorkGenerator & work_generator)
{
  // Generate the work function for the DAG based on id
  for (auto & itr : dag.node_list_) {
    itr.second->work(work_generator(itr.first));
  }

  future_ = std::make_shared<tf::Future<void>>(executor_->run(*dag.taskflow_));
  return future_->share();
}

void DAGExecutor::cancel()
{
  future_->cancel();
}

}  // namespace rmf_scheduler
