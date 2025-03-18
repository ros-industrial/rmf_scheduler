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

#include "pybind11_json/pybind11_json.hpp"
#include "rmf2_scheduler_py/data/json_serializer.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

template<typename T>
nlohmann::json serialize(const T & data)
{
  return nlohmann::json(data);
}

template<typename T>
void deserialize(const nlohmann::json & j, T & data)
{
  data = j.template get<T>();
}

void init_json_serializer_py(py::module & m)
{
  py::module m_data = m.def_submodule("data");
  m_data.def_submodule("json_serializer")
  .def("serialize", &serialize<rmf2_scheduler::data::Time>)
  .def("serialize", &serialize<rmf2_scheduler::data::Duration>)
  .def("serialize", &serialize<rmf2_scheduler::data::Event>)
  .def("serialize", &serialize<rmf2_scheduler::data::Graph>)
  .def("serialize", &serialize<rmf2_scheduler::data::Process>)

  .def("deserialize", &deserialize<rmf2_scheduler::data::Time>)
  .def("deserialize", &deserialize<rmf2_scheduler::data::Duration>)
  .def("deserialize", &deserialize<rmf2_scheduler::data::Event>)
  .def("deserialize", &deserialize<rmf2_scheduler::data::Process>)
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
