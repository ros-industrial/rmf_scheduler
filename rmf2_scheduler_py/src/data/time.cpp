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
#include "rmf2_scheduler_py/data/time.hpp"
#include "rmf2_scheduler/data/time.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_time_py(py::module & m)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  py::module m_data = m.def_submodule("data");

  py::class_<Time>(
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
  .def(py::self + Duration())
  .def(py::self += Duration())
  .def(py::self - py::self)
  .def(py::self - Duration())
  .def(py::self -= Duration())
  .def(
    "nanoseconds",
    &Time::nanoseconds
  )
  .def_static(
    "max",
    &Time::max
  )
  .def(
    "seconds",
    &Time::seconds
  )
  .def(
    "to_datetime",
    &Time::to_chrono<std::chrono::system_clock::time_point>
  )
  .def(
    "to_localtime",
    py::overload_cast<const std::string &, const std::string &>(
      &Time::to_localtime,
      py::const_
    ),
    py::arg("timezone") = "UTC",
    py::arg("fmt") = "%b %d %H:%M:%S %Y"
  )
  .def(
    "to_ISOtime",
    &Time::to_ISOtime
  )
  .def_static(
    "from_localtime",
    py::overload_cast<
      const std::string &,
      const std::string &,
      const std::string &
    >(
      &Time::from_localtime
    ),
    py::arg("localtime"),
    py::arg("timezone") = "UTC",
    py::arg("fmt") = "%b %d %H:%M:%S %Y"
  )
  .def_static(
    "from_ISOtime",
    &Time::from_ISOtime
  )

  // Serialization
  .def(
    "json",
    [](
      Time & self
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
      return j.template get<Time>();
    }
  )
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
