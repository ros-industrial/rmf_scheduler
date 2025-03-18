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

#include "rmf2_scheduler_py/data/graph.hpp"
#include "rmf2_scheduler/data/graph.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_graph_py(py::module & m)
{
  py::module m_data = m.def_submodule("data");

  py::class_<rmf2_scheduler::data::Graph, rmf2_scheduler::data::Graph::Ptr>(
    m_data,
    "Graph",
    R"(
    Graph class
    )"
  )
  .def(py::init<>())
  .def(
    "add_node",
    &rmf2_scheduler::data::Graph::add_node
  )
  .def(
    "add_edge",
    &rmf2_scheduler::data::Graph::add_edge,
    py::arg("source"),
    py::arg("destination"),
    py::arg("edge") = rmf2_scheduler::data::Edge("hard")
  )
  .def(
    "update_node",
    &rmf2_scheduler::data::Graph::update_node
  )
  .def(
    "delete_edge",
    &rmf2_scheduler::data::Graph::delete_edge,
    py::arg("source"),
    py::arg("destination")
  )
  .def(
    "delete_node",
    &rmf2_scheduler::data::Graph::delete_node
  )
  .def(
    "prune",
    &rmf2_scheduler::data::Graph::prune
  )
  .def(
    "has_node",
    &rmf2_scheduler::data::Graph::has_node
  )
  .def(
    "get_node",
    &rmf2_scheduler::data::Graph::get_node
  )
  .def(
    "get_all_nodes",
    &rmf2_scheduler::data::Graph::get_all_nodes
  )
  .def(
    "empty",
    &rmf2_scheduler::data::Graph::empty
  )
  .def(
    "dump",
    [](const rmf2_scheduler::data::Graph & self) {
      std::ostringstream oss;
      self.dump(oss);
      return oss.str();
    }
  )
  .def(py::self == py::self)
  .def(py::self != py::self)
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
