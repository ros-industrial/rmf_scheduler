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

#include "rmf2_scheduler_py/data/duration.hpp"
#include "rmf2_scheduler/data/duration.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_duration_py(py::module & m)
{
  py::module m_data = m.def_submodule("data");

  py::class_<rmf2_scheduler::data::Duration>(
    m_data,
    "Duration",
    R"(
    Duration class
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
    py::init<const std::chrono::system_clock::duration &>(),
    py::arg("duration")
  )
  .def(py::self == py::self)
  .def(py::self != py::self)
  .def(py::self < py::self)
  .def(py::self <= py::self)
  .def(py::self >= py::self)
  .def(py::self > py::self)
  .def(py::self + py::self)
  .def(py::self += py::self)
  .def(py::self - py::self)
  .def(py::self -= py::self)
  .def(py::self * double())
  .def(py::self *= double())
  .def_static(
    "max",
    &rmf2_scheduler::data::Duration::max
  )
  .def(
    "nanoseconds",
    &rmf2_scheduler::data::Duration::nanoseconds
  )
  .def(
    "seconds",
    &rmf2_scheduler::data::Duration::seconds
  )
  .def(
    "to_timedelta",
    &rmf2_scheduler::data::Duration::to_chrono<std::chrono::system_clock::duration>
  )
  .def_static(
    "from_seconds",
    &rmf2_scheduler::data::Duration::from_seconds,
    py::arg("seconds")
  )
  .def_static(
    "from_nanoseconds",
    &rmf2_scheduler::data::Duration::from_nanoseconds,
    py::arg("nanoseconds")
  )
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
