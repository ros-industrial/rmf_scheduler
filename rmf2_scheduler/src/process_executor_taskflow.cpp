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

#include "rmf2_scheduler/log.hpp"
#include "rmf2_scheduler/macros.hpp"
#include "rmf2_scheduler/process_executor_taskflow.hpp"

namespace rmf2_scheduler
{

bool TaskflowProcessExecutor::run_async(
  const data::Process::ConstPtr & process,
  const std::vector<data::Task::ConstPtr> & tasks,
  std::string & error
)
{
  LOG_DEBUG(
    "TaskflowProcessExecutor run_async: starting run_async for process: %s",
    process->id.c_str()
  );

  if (process->graph.size() != tasks.size()) {
    error =
      "TaskflowProcessExecutor run_async: "
      "number of tasks does not match number of nodes in the process graph.";
    return false;
  }

  //  Set up a taskflow.
  auto tf_executor = std::make_shared<tf::Executor>(concurrency_);
  auto taskflow = std::make_shared<tf::Taskflow>();

  // Provides lookup of input tasks.
  auto input_task_map = task_vector_to_map(tasks);
  // Tracks tf::Tasks for subsequent creation of dependencies in the taskflow.
  std::unordered_map<std::string, std::shared_ptr<tf::Task>> tf_task_map;

  // Emplace nodes to the taskflow
  process->graph.for_each_node_ordered(
    [&, this](rmf2_scheduler::data::Node::Ptr node)
    {
      auto work_function = make_work_function(
        process->id, node->id(), input_task_map.at(node->id())
      );
      tf::Task task = taskflow->emplace(work_function).name(node->id());
      tf_task_map.insert({node->id(), std::make_shared<tf::Task>(task)});
    });

  // Add dependencies to the taskflow.
  for (const auto & taskmap_entry : tf_task_map) {
    auto node = process->graph.get_node(taskmap_entry.first);

    for (const auto & edge : node->outbound_edges()) {
      auto successor_task = tf_task_map.at(edge.first);
      taskmap_entry.second->precede(*successor_task);
    }
  }

  // Begin running the taskflow
  auto entry = std::make_shared<TaskflowEntry>(process, tf_executor, taskflow);
  entry->future = tf_executor->run(*taskflow);

  {  // Lock
    std::lock_guard lk(mtx_);
    taskflows_.insert({process->id, entry});
  }  // Unlock

  return true;
}

std::shared_future<void> TaskflowProcessExecutor::cancel(const data::Process::ConstPtr & process)
{
  return cancel(process->id);
}

std::shared_future<void> TaskflowProcessExecutor::cancel(const std::string & process_id)
{
  {  // Lock
    std::lock_guard lk(mtx_);

    auto taskflow_itr = taskflows_.find(process_id);
    if (taskflow_itr == taskflows_.end()) {
      // Invalid process ID
      return std::shared_future<void>();
    }

    // Return shared_future
    taskflow_itr->second->future.cancel();
    return taskflow_itr->second->future.share();
  }  // Unlock
}


const std::unordered_map<std::string,
  data::Task::ConstPtr>
TaskflowProcessExecutor::task_vector_to_map(
  const std::vector<data::Task::ConstPtr> & tasks)
{
  std::unordered_map<std::string, data::Task::ConstPtr> map;
  for (auto & task : tasks) {
    auto result = map.insert({task->id, task});
    if (!result.second) {
      // TODO(anyone): handle error
      LOG_ERROR("Duplicate task ID: %s present in process.", task->id.c_str());
    }
  }
  return map;
}

std::function<void()> TaskflowProcessExecutor::make_work_function(
  const std::string & process_id,
  const std::string & node_id,
  const rmf2_scheduler::data::Task::ConstPtr & task
)
{
  return [process_id, node_id, task, this]() -> void
         {
           LOG_DEBUG(
             "TaskflowProcessExecutor work_function: "
             "running work function for %s..", node_id.c_str()
           );
           ExecutorData tem_debug_data;
           std::future<bool> fut;
           std::string tem_error;

           bool run = tem_->run_async(
             task, &tem_debug_data, &fut,
             tem_error
           );

           if (!run) {
             LOG_ERROR(
               "TaskflowProcessExecutor work_function failed: executing task [%s]: %s",
               node_id.c_str(), tem_error.c_str()
             );
             LOG_DEBUG(
               "TaskflowProcessExecutor work_function: Debug data for task [%s]: %s",
               node_id.c_str(), tem_debug_data.get_data_as_string().c_str()
             );

             LOG_ERROR(
               "TaskflowProcessExecutor work_function failed: "
               "Cancelling rest of workflow [%s]",
               process_id.c_str()
             );

             {  // Lock
               std::lock_guard lk(mtx_);
               taskflows_.at(process_id)->future.cancel();
             }  // Unlock

             return;
           }

           LOG_DEBUG(
             "TaskflowProcessExecutor work_function: "
             "Waiting for result of task [%s]", node_id.c_str()
           );
           fut.wait();

           LOG_DEBUG(
             "TaskflowProcessExecutor work_function: "
             "Received result for task [%s]", node_id.c_str()
           );
           fut.get();
         };
}

void TaskflowProcessExecutor::wait_for_all()
{
  for (auto taskflow : taskflows_) {
    if (taskflow.second->future.valid()) {
      taskflow.second->future.get();
    }
  }
}

TaskflowProcessExecutor::~TaskflowProcessExecutor()
{
  // Destructor definition, even if it is empty
  // TODO(anyone): safe shutdown
}


}  // namespace rmf2_scheduler
