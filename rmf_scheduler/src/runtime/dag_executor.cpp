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

#include "rmf_scheduler/runtime/dag_executor.hpp"
#include "taskflow/taskflow.hpp"

namespace rmf_scheduler
{

namespace runtime
{

DAGExecutor::Work DAGExecutor::EmptyWork()
{
  return []() {};
}

DAGExecutor::DAGExecutor(unsigned int concurrency)
: executor_(std::make_unique<tf::Executor>(concurrency))
{
}

DAGExecutor::~DAGExecutor()
{
  cancel();
  cancel_future_->get();
}

std::shared_future<void> DAGExecutor::run(
  const data::DAG::Description & dag,
  WorkGenerator work_generator)
{
  // Deep copy the DAG
  dag_ = std::make_unique<data::DAG>(dag);

  // Generate the work function for the DAG based on id
  for (auto & itr : dag_->node_list_) {
    itr.second->work(work_generator(itr.first));
  }

  cancel_handler_ = std::make_shared<tf::Future<void>>(executor_->run(*dag_->taskflow_));
  future_ = cancel_handler_->share();
  return future_;
}

void DAGExecutor::cancel_and_next(const data::DAG::Description & dag, WorkGenerator work_generator)
{
  std::lock_guard<std::mutex> lock(next_dag_mtx_);

  // Deep copy the DAG
  next_dag_ = std::make_unique<data::DAG>(dag);

  // Generate the work function for the DAG based on id
  for (auto & itr : next_dag_->node_list_) {
    itr.second->work(work_generator(itr.first));
  }

  // Skip if canceling in progress
  if (cancel_future_ &&
    cancel_future_->wait_for(std::chrono::microseconds(1)) !=
    std::future_status::ready)
  {
    return;
  }

  cancel_future_ = std::make_shared<std::future<void>>(
    std::async(
      std::bind(&DAGExecutor::_cancel_and_next, this)));
}

void DAGExecutor::cancel()
{
  std::lock_guard<std::mutex> lock(next_dag_mtx_);

  // Delete the next DAG
  next_dag_.reset();

  // Skip if canceling in progress
  if (cancel_future_ &&
    cancel_future_->wait_for(std::chrono::microseconds(1)) !=
    std::future_status::ready)
  {
    return;
  }

  cancel_future_ = std::make_shared<std::future<void>>(
    std::async(
      std::bind(&DAGExecutor::_cancel_and_next, this)));
}

bool DAGExecutor::ongoing() const
{
  std::lock_guard<std::mutex> lock(next_dag_mtx_);
  // Skip if canceling in progress
  if (cancel_future_ &&
    cancel_future_->wait_for(std::chrono::microseconds(1)) ==
    std::future_status::timeout)
  {
    return true;
  }
  if (future_.valid() &&
    future_.wait_for(std::chrono::microseconds(1)) ==
    std::future_status::timeout)
  {
    return true;
  }

  return false;
}

void DAGExecutor::_cancel_and_next()
{
  if (future_.valid()) {
    cancel_handler_->cancel();
    future_.get();
  }

  // Lock to check if there is a next DAG
  std::lock_guard<std::mutex> lock(next_dag_mtx_);
  if (next_dag_) {
    dag_ = std::move(next_dag_);
    cancel_handler_ = std::make_shared<tf::Future<void>>(executor_->run(*dag_->taskflow_));
    future_ = cancel_handler_->share();
  }
}


}  // namespace runtime

}  // namespace rmf_scheduler
