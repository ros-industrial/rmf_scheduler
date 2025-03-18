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

#include "rmf2_scheduler_py/data/process.hpp"
#include "rmf2_scheduler/data/process.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_process_py(py::module & m)
{
  py::module m_data = m.def_submodule("data");

  py::class_<rmf2_scheduler::data::Process, rmf2_scheduler::data::Process::Ptr>(
    m_data,
    "Process",
    R"(
    Additional information for a Task
    )"
  )
  .def(py::init<>())
  .def_readwrite(
    "id",
    &rmf2_scheduler::data::Process::id
  )
  .def_readwrite(
    "graph",
    &rmf2_scheduler::data::Process::graph
  )
  .def(py::self == py::self)
  .def(py::self != py::self)
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
