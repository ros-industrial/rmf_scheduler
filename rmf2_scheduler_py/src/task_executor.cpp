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

#include "rmf2_scheduler_py/task_executor.hpp"
#include "rmf2_scheduler/task_executor.hpp"
#include "rmf2_scheduler/executor_data.hpp"

namespace rmf2_scheduler
{

class PyTaskExecutor : public TaskExecutor
{
public:
  using TaskExecutor::TaskExecutor;

  /// Trampoline
  bool build(
    const data::Task::ConstPtr & task,
    ExecutorData * data,
    std::string & error
  ) override
  {
    pybind11::gil_scoped_acquire gil;  // Acquire the GIL while in this scope.

    // Try to look up the overridden method on the Python side.
    pybind11::function override = pybind11::get_override(this, "build");

    if (!override) {
      error = "TaskExecutor build failed: cannot find defined Python function";
      return false;
    }

    auto obj = override (task);
    if (!py::isinstance<py::tuple>(obj)) {
      error = "TaskExecutor build failed: Invalid Python return type.";
      return false;
    }

    py::tuple tuple_obj = obj;
    if (py::len(tuple_obj) >= 2) {
      bool result = tuple_obj[0].cast<bool>();
      if (!result) {
        error = tuple_obj[1].cast<std::string>();
        return false;
      }

      if (py::len(tuple_obj) >= 3) {
        try {
          const ExecutorData * temp_data = tuple_obj[2].cast<ExecutorData *>();
          data->set_data(
            temp_data->get_data().data(),
            temp_data->get_data().size()
          );
          return true;
        } catch (const py::cast_error & e) {
          error =
            std::string("TaskExecutor build failed: Unable to unpack data.\n") +
            e.what();
          return false;
        }
      } else {
        return true;
      }
    }

    error = "TaskExecutor build failed: Invalid number of returns";
    return false;
  }

  bool start(
    const std::string & id,
    const ExecutorData * data,
    std::string & error
  ) override
  {
    pybind11::gil_scoped_acquire gil;  // Acquire the GIL while in this scope.

    // Try to look up the overridden method on the Python side.
    pybind11::function override = pybind11::get_override(this, "start");

    if (!override) {
      error = "TaskExecutor start failed: cannot find defined Python function";
      return false;
    }

    auto obj = override (id, data);
    if (!py::isinstance<py::tuple>(obj)) {
      error = "TaskExecutor start failed: Invalid Python return type.";
      return false;
    }

    py::tuple tuple_obj = obj;
    if (py::len(tuple_obj) != 2) {
      error = "TaskExecutor start failed: Invalid number of returns";
      return false;
    }

    bool result = tuple_obj[0].cast<bool>();
    if (!result) {
      error = tuple_obj[1].cast<std::string>();
      return false;
    }

    return true;
  }
};

}  // namespace rmf2_scheduler

namespace rmf2_scheduler_py
{
void init_task_executor_py(py::module & m)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  py::class_<TaskExecutor, PyTaskExecutor, TaskExecutor::Ptr>(
    m,
    "TaskExecutor"
  )
  .def(py::init<>())
  .def(
    "build",
    [](
      TaskExecutor & self,
      const data::Task::ConstPtr & task
    ) {
      static ExecutorData data;
      std::string error;
      bool result = self.build(task, &data, error);
      return std::make_tuple(result, error, &data);
    }
  )
  .def(
    "start",
    [](
      TaskExecutor & self,
      const std::string & id,
      const ExecutorData * data
    ) {
      std::string error;
      bool result = self.start(id, data, error);
      return std::make_tuple(result, error);
    }
  )
  ;
}

}  // namespace rmf2_scheduler_py
