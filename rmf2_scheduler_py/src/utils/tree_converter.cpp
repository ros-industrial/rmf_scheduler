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
#include <pybind11/functional.h>

#include "rmf2_scheduler_py/utils/tree_converter.hpp"
#include "rmf2_scheduler/utils/tree_converter.hpp"

namespace rmf2_scheduler_py
{

namespace utils
{

void init_tree_converter_py(py::module & m)
{
  py::module m_data = m.def_submodule("utils");

  py::class_<rmf2_scheduler::utils::TreeConversion>(
    m_data,
    "TreeConversion",
    R"(
    DAG to Behavior Tree
    )"
  )
  .def(
    py::init<>()
  )
  .def(
    "convert_to_tree",
    py::overload_cast<
      const rmf2_scheduler::data::Graph &,
      const rmf2_scheduler::utils::TreeConversion::MakeTreeCallback &
    >(
      &rmf2_scheduler::utils::TreeConversion::convert_to_tree
    )
  )
  .def(
    "convert_to_tree",
    py::overload_cast<
      const std::string &,
      const rmf2_scheduler::data::Graph &,
      const rmf2_scheduler::utils::TreeConversion::MakeTreeCallback &
    >(
      &rmf2_scheduler::utils::TreeConversion::convert_to_tree
    )
  )
  ;
}

}  // namespace utils

}  // namespace rmf2_scheduler_py
