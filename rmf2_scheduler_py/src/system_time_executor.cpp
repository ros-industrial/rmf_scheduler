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
#include <pybind11/functional.h>

#include "rmf2_scheduler_py/system_time_executor.hpp"
#include "rmf2_scheduler/system_time_executor.hpp"

namespace rmf2_scheduler_py
{

void init_system_time_executor_py(py::module & m)
{
  py::class_<rmf2_scheduler::SystemTimeAction>(
    m,
    "SystemTimeAction",
    R"(
    SystemTimeAction
    )"
  )
  .def(py::init<>())
  .def_readwrite(
    "time",
    &rmf2_scheduler::SystemTimeAction::time
  )
  .def_readwrite(
    "work",
    &rmf2_scheduler::SystemTimeAction::work
  )
  ;

  py::class_<rmf2_scheduler::SystemTimeExecutor, rmf2_scheduler::SystemTimeExecutor::Ptr>(
    m,
    "SystemTimeExecutor",
    R"(
    SystemTimeExecutor Class
    )"
  )
  .def(py::init<>())
  .def(
    "add_action",
    &rmf2_scheduler::SystemTimeExecutor::add_action
  )
  .def(
    "delete_action",
    &rmf2_scheduler::SystemTimeExecutor::delete_action
  )
  .def(
    "spin",
    [](rmf2_scheduler::SystemTimeExecutor & self) {
      py::gil_scoped_release release;
      self.spin();
      py::gil_scoped_acquire acquire;
    }
  )
  .def(
    "stop",
    &rmf2_scheduler::SystemTimeExecutor::stop
  )
  ;
}

}  // namespace rmf2_scheduler_py
