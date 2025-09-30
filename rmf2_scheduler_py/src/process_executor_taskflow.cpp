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

#include "rmf2_scheduler_py/process_executor_taskflow.hpp"
#include "rmf2_scheduler/process_executor_taskflow.hpp"
#include "rmf2_scheduler/process_executor.hpp"

namespace rmf2_scheduler_py
{
void init_process_executor_taskflow_py(py::module & m)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  py::class_<TaskflowProcessExecutor, ProcessExecutor, TaskflowProcessExecutor::Ptr>(
    m,
    "TaskflowProcessExecutor"
  )
  .def(
    py::init<
      TaskExecutorManager::Ptr,
      unsigned int
    >(),
    py::arg("tem"),
    py::arg("concurrency") = std::thread::hardware_concurrency()
  )
  ;
}

}  // namespace rmf2_scheduler_py
