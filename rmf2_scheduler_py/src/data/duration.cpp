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

#include "pybind11_json/pybind11_json.hpp"
#include "rmf2_scheduler_py/data/duration.hpp"
#include "rmf2_scheduler/data/duration.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_duration_py(py::module & m)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  py::module m_data = m.def_submodule("data");

  py::class_<Duration>(
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
    &Duration::max
  )
  .def(
    "nanoseconds",
    &Duration::nanoseconds
  )
  .def(
    "seconds",
    &Duration::seconds
  )
  .def(
    "to_timedelta",
    &Duration::to_chrono<std::chrono::system_clock::duration>
  )
  .def_static(
    "from_seconds",
    &Duration::from_seconds,
    py::arg("seconds")
  )
  .def_static(
    "from_nanoseconds",
    &Duration::from_nanoseconds,
    py::arg("nanoseconds")
  )

  // Serialization
  .def(
    "json",
    [](
      Duration & self
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
      return j.template get<Duration>();
    }
  )
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
