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
#include "rmf2_scheduler_py/data/series.hpp"
#include "rmf2_scheduler/data/occurrence.hpp"
#include "rmf2_scheduler/data/series.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_series_py(py::module & m)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  py::module m_data = m.def_submodule("data");

  py::class_<Series, Series::Ptr>(
    m_data,
    "Series",
    R"(
    Additional information for a Process
    )"
  )
  .def(py::init<>())
  .def(
    py::init<
      const std::string &,
      const std::string &,
      const Occurrence &,
      const std::string &,
      const std::string &,
      const Time &
    >(),
    py::arg("id"),
    py::arg("type"),
    py::arg("occurrence"),
    py::arg("cron"),
    py::arg("timezone"),
    py::arg("until") = Time::max()
  )
  .def(py::self == py::self)
  .def(py::self != py::self)
  .def(
    "id",
    &Series::id
  )

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
      return j.template get<Series>();
    }
  )
  ;
}
}  // namespace data

}  // namespace rmf2_scheduler_py
