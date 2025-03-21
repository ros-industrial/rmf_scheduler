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
#include "rmf2_scheduler_py/data/event.hpp"
#include "rmf2_scheduler/data/event.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_event_py(py::module & m)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  py::module m_data = m.def_submodule("data");

  py::class_<Event, Event::Ptr>(
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
      const Time &
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
      const Time &,
      const Duration &,
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
    &Event::id
  )
  .def_readwrite(
    "type",
    &Event::type
  )
  .def_readwrite(
    "description",
    &Event::description
  )
  .def_readwrite(
    "start_time",
    &Event::start_time
  )
  .def_readwrite(
    "duration",
    &Event::duration
  )
  .def_readwrite(
    "series_id",
    &Event::series_id
  )
  .def_readwrite(
    "process_id",
    &Event::process_id
  )
  .def(py::self == py::self)
  .def(py::self != py::self)

  // JSON Serialization
  .def(
    "json",
    [](
      Event & self
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
      return j.template get<Event>();
    }
  )
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
