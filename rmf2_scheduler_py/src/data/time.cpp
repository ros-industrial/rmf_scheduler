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

#include <pybind11/chrono.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <chrono>

#include "rmf2_scheduler_py/data/time.hpp"
#include "rmf2_scheduler/data/time.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_time_py(py::module & m)
{
  py::module m_data = m.def_submodule("data");

  py::class_<rmf2_scheduler::data::Time>(
    m_data,
    "Time",
    R"(
    Time class
    )"
  )
  .def(py::init<>())
  .def(
    py::init<int32_t, uint32_t>(),
    py::arg("seconds"),
    py::arg("nanoseconds")
  )
  .def(
    py::init<int64_t>(),
    py::arg("nanoseconds")
  )
  .def(
    py::init<const std::chrono::system_clock::time_point &>(),
    py::arg("time_point")
  )
  .def(py::self == py::self)
  .def(py::self != py::self)
  .def(py::self < py::self)
  .def(py::self <= py::self)
  .def(py::self >= py::self)
  .def(py::self > py::self)
  .def(py::self + rmf2_scheduler::data::Duration())
  .def(py::self += rmf2_scheduler::data::Duration())
  .def(py::self - py::self)
  .def(py::self - rmf2_scheduler::data::Duration())
  .def(py::self -= rmf2_scheduler::data::Duration())
  .def(
    "nanoseconds",
    &rmf2_scheduler::data::Time::nanoseconds
  )
  .def_static(
    "max",
    &rmf2_scheduler::data::Time::max
  )
  .def(
    "seconds",
    &rmf2_scheduler::data::Time::seconds
  )
  .def(
    "to_datetime",
    &rmf2_scheduler::data::Time::to_chrono<std::chrono::system_clock::time_point>
  )
  .def(
    "to_localtime",
    &rmf2_scheduler::data::Time::to_localtime,
    py::arg("fmt") = "%b %d %H:%M:%S %Y"
  )
  .def(
    "to_ISOtime",
    &rmf2_scheduler::data::Time::to_ISOtime
  )
  .def_static(
    "from_localtime",
    &rmf2_scheduler::data::Time::from_localtime,
    py::arg("localtime"),
    py::arg("fmt") = "%b %d %H:%M:%S %Y"
  )
  .def_static(
    "from_ISOtime",
    &rmf2_scheduler::data::Time::from_ISOtime
  )
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
