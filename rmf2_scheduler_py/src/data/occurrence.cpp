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
#include "rmf2_scheduler/data/occurrence.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_occurrence_py(py::module & m)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  py::module m_data = m.def_submodule("data");

  py::class_<Occurrence>(
    m_data,
    "Occurrence",
    R"(
    Information for a series occurrence
    )"
  )
  .def(py::init<>())
  .def(
    py::init<
      const Time &,
      const std::string &
    >(),
    py::arg("_time"),
    py::arg("id")
  )
  .def(py::self == py::self)
  .def(py::self != py::self)

  // Serialization
  .def(
    "json",
    [](
      Series & self
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
      return j.template get<Occurrence>();
    }
  )
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
