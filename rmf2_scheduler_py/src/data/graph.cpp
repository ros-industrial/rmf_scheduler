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

#include <sstream>
#include <ostream>

#include "pybind11_json/pybind11_json.hpp"
#include "rmf2_scheduler_py/data/graph.hpp"
#include "rmf2_scheduler/data/graph.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_graph_py(py::module & m)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  py::module m_data = m.def_submodule("data");

  py::class_<Graph, Graph::Ptr>(
    m_data,
    "Graph",
    R"(
    Graph class
    )"
  )
  .def(py::init<>())
  .def(
    "add_node",
    &Graph::add_node
  )
  .def(
    "add_edge",
    &Graph::add_edge,
    py::arg("source"),
    py::arg("destination"),
    py::arg("edge") = Edge("hard")
  )
  .def(
    "update_node",
    &Graph::update_node
  )
  .def(
    "delete_edge",
    &Graph::delete_edge,
    py::arg("source"),
    py::arg("destination")
  )
  .def(
    "delete_node",
    &Graph::delete_node
  )
  .def(
    "prune",
    &Graph::prune
  )
  .def(
    "has_node",
    &Graph::has_node
  )
  .def(
    "get_node",
    &Graph::get_node
  )
  .def(
    "get_all_nodes",
    &Graph::get_all_nodes
  )
  .def(
    "empty",
    &Graph::empty
  )
  .def(
    "dump",
    [](const Graph & self) {
      std::ostringstream oss;
      self.dump(oss);
      return oss.str();
    }
  )
  .def(py::self == py::self)
  .def(py::self != py::self)

  // Serialization
  .def(
    "json",
    [](
      Graph & self
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
      return j.template get<Graph>();
    }
  )
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
