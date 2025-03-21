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

#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include "pybind11_json/pybind11_json.hpp"
#include "rmf2_scheduler_py/data/task.hpp"
#include "rmf2_scheduler/data/task.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_task_py(py::module & m)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  py::module m_data = m.def_submodule("data");

  py::class_<Task, Task::Ptr,
    Event>(
    m_data,
    "Task",
    R"(
    Additional information for a Task
    )"
    )
  .def(py::init<>())
  .def(
    py::init<
      const std::string &,
      const std::string &,
      const Time &,
      const std::string &
    >(),
    py::arg("id"),
    py::arg("type"),
    py::arg("start_time"),
    py::arg("status")
  )
  .def(
    py::init<
      const Event &,
      const std::string &
    >(),
    py::arg("event"),
    py::arg("status")
  )
  .def(
    py::init<
      const std::string &,
      const std::string &,
      const std::string &,
      const Time &,
      const Duration &,
      const std::string &,
      const std::string &,
      const std::string &,
      const Time &,
      const std::string &,
      const Time &,
      const Duration &,
      const Duration &,
      const Time &,
      const Duration &,
      const nlohmann::json &
    >(),
    py::arg("id"),
    py::arg("type"),
    py::arg("description"),
    py::arg("start_time"),
    py::arg("duration"),
    py::arg("series_id"),
    py::arg("process_id"),
    py::arg("resource_id"),
    py::arg("deadline"),
    py::arg("status"),
    py::arg("planned_start_time"),
    py::arg("planned_duration"),
    py::arg("estimated_duration"),
    py::arg("actual_start_time"),
    py::arg("actual_duration"),
    py::arg("task_details")
  )
  .def(
    py::init<
      const Event &,
      const std::string &,
      const Time &,
      const std::string &,
      const Time &,
      const Duration &,
      const Duration &,
      const Time &,
      const Duration &,
      const nlohmann::json &
    >(),
    py::arg("event"),
    py::arg("resource_id"),
    py::arg("deadline"),
    py::arg("status"),
    py::arg("planned_start_time"),
    py::arg("planned_duration"),
    py::arg("estimated_duration"),
    py::arg("actual_start_time"),
    py::arg("actual_duration"),
    py::arg("task_details")
  )
  .def_readwrite(
    "resource_id",
    &Task::resource_id
  )
  .def_readwrite(
    "deadline",
    &Task::deadline
  )
  .def_readwrite(
    "status",
    &Task::status
  )
  .def_readwrite(
    "planned_start_time",
    &Task::planned_start_time
  )
  .def_readwrite(
    "planned_duration",
    &Task::planned_duration
  )
  .def_readwrite(
    "estimated_duration",
    &Task::estimated_duration
  )
  .def_readwrite(
    "actual_start_time",
    &Task::actual_start_time
  )
  .def_readwrite(
    "actual_duration",
    &Task::actual_duration
  )
  .def_readwrite(
    "task_details",
    &Task::task_details
  )
  .def(py::self == py::self)
  .def(py::self != py::self)

  // Serialization
  .def(
    "json",
    [](
      Task & self
    )
    {
      return nlohmann::json(self);
    }
  )
  .def_static(
    "from_json",
    [](
      const nlohmann::json & j
    )
    {
      return j.template get<Task>();
    }
  )
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
