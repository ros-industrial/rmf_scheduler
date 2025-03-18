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

#include "rmf2_scheduler_py/data/event.hpp"
#include "rmf2_scheduler/data/event.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_event_py(py::module & m)
{
  py::module m_data = m.def_submodule("data");

  py::class_<rmf2_scheduler::data::Event, rmf2_scheduler::data::Event::Ptr>(
    m_data,
    "Event",
    R"(
    Basic information about the Event
    )"
  )
  .def(py::init<>())
  .def(
    py::init<
      const std::string &,
      const std::string &,
      const rmf2_scheduler::data::Time &
    >(),
    py::arg("id"),
    py::arg("type"),
    py::arg("start_time")
  )
  .def(
    py::init<
      const std::string &,
      const std::string &,
      const std::string &,
      const rmf2_scheduler::data::Time &,
      const rmf2_scheduler::data::Duration &,
      const std::string &,
      const std::string &
    >(),
    py::arg("id"),
    py::arg("type"),
    py::arg("description"),
    py::arg("start_time"),
    py::arg("duration"),
    py::arg("series_id"),
    py::arg("process_id")
  )
  .def_readwrite(
    "id",
    &rmf2_scheduler::data::Event::id
  )
  .def_readwrite(
    "type",
    &rmf2_scheduler::data::Event::type
  )
  .def_readwrite(
    "description",
    &rmf2_scheduler::data::Event::description
  )
  .def_readwrite(
    "start_time",
    &rmf2_scheduler::data::Event::start_time
  )
  .def_readwrite(
    "duration",
    &rmf2_scheduler::data::Event::duration
  )
  .def_readwrite(
    "series_id",
    &rmf2_scheduler::data::Event::series_id
  )
  .def_readwrite(
    "process_id",
    &rmf2_scheduler::data::Event::process_id
  )
  .def(py::self == py::self)
  .def(py::self != py::self)
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
