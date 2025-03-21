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

#include <pybind11/stl.h>

#include "rmf2_scheduler_py/task_executor_manager.hpp"
#include "rmf2_scheduler/task_executor_manager.hpp"
#include "rmf2_scheduler/executor_data.hpp"
#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler_py
{

void init_task_executor_manager_py(py::module & m)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  py::class_<TaskExecutorManager, TaskExecutorManager::Ptr>(
    m,
    "TaskExecutorManager"
  )
  .def(py::init<>())
  .def(
    "load",
    &TaskExecutorManager::load
  )
  .def(
    "unload",
    &TaskExecutorManager::unload
  )
  .def(
    "is_runnable",
    &TaskExecutorManager::is_runnable
  )
  .def(
    "dry_run",
    [](
      TaskExecutorManager & self,
      const data::Task::ConstPtr & task
    ) {
      static ExecutorData data;
      std::string error;
      bool result = self.dry_run(task, &data, error);
      return py::make_tuple(result, error, &data);
    }
  )
  .def(
    "update",
    &TaskExecutorManager::update
  )
  .def(
    "notify_completion",
    &TaskExecutorManager::notify_completion
  )
  .def(
    "run_async",
    [](
      TaskExecutorManager & self,
      const data::Task::ConstPtr & task
    ) {
      std::string error;
      bool result = self.run_async(task, nullptr, nullptr, error);
      return py::make_tuple(result, error);
    }
  )
  ;
}

}  // namespace rmf2_scheduler_py
