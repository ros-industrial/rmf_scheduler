// Copyright 2025 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__PROCESS_EXECUTOR_TASKFLOW_HPP_
#define RMF2_SCHEDULER__PROCESS_EXECUTOR_TASKFLOW_HPP_

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rmf2_scheduler/executor_data.hpp"
#include "rmf2_scheduler/macros.hpp"
#include "rmf2_scheduler/process_executor.hpp"
#include "rmf2_scheduler/task_executor_manager.hpp"

#include <taskflow/taskflow.hpp>

namespace rmf2_scheduler
{

struct TaskflowEntry
{
  data::Process::ConstPtr process;
  std::shared_ptr<tf::Executor> tf_executor;
  std::shared_ptr<tf::Taskflow> taskflow;
  tf::Future<void> future;

  TaskflowEntry(
    const data::Process::ConstPtr & proc,
    std::shared_ptr<tf::Executor> tf_exec,
    std::shared_ptr<tf::Taskflow> tf
  )
  : process(proc), tf_executor(std::move(tf_exec)), taskflow(std::move(tf)) {}
};

class TaskflowProcessExecutor : public ProcessExecutor
{
public:
  RS_SMART_PTR_DEFINITIONS(TaskflowProcessExecutor)

  explicit TaskflowProcessExecutor(
    std::shared_ptr<TaskExecutorManager> tem,
    unsigned int concurrency = std::thread::hardware_concurrency()
  )
  : ProcessExecutor(), tem_(tem), concurrency_(concurrency) {}

  virtual ~TaskflowProcessExecutor();

  bool run_async(
    const data::Process::ConstPtr & process,
    const std::vector<data::Task::ConstPtr> & tasks,
    std::string & error) override;

  std::shared_future<void> cancel(const data::Process::ConstPtr & process);

  std::shared_future<void> cancel(const std::string & process_id);

  void wait_for_all();

private:
  RS_DISABLE_COPY(TaskflowProcessExecutor)

  std::shared_ptr<TaskExecutorManager> tem_;
  unsigned int concurrency_;
  std::unordered_map<std::string, std::shared_ptr<TaskflowEntry>> taskflows_;
  std::mutex mtx_;

  const std::unordered_map<std::string, data::Task::ConstPtr> task_vector_to_map(
    const std::vector<data::Task::ConstPtr> & tasks);

  std::function<void()> make_work_function(
    const std::string & process_id,
    const std::string & node_id,
    const rmf2_scheduler::data::Task::ConstPtr & task
  );

  friend class TestProcessExecutorTaskflow;
};

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__PROCESS_EXECUTOR_TASKFLOW_HPP_
