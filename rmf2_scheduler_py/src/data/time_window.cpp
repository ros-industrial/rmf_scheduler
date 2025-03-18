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

#include "rmf2_scheduler_py/data/time_window.hpp"
#include "rmf2_scheduler/data/time_window.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_time_window_py(py::module & m)
{
  py::module m_data = m.def_submodule("data");

  py::class_<rmf2_scheduler::data::TimeWindow>(
    m_data,
    "TimeWindow",
    R"(
    Time window
    )"
  )
  .def(py::init<>())
  .def_readwrite(
    "start",
    &rmf2_scheduler::data::TimeWindow::start
  )
  .def_readwrite(
    "end",
    &rmf2_scheduler::data::TimeWindow::end
  )
  .def(py::self == py::self)
  .def(py::self != py::self)
  ;
}

}  // namespace data
}  // namespace rmf2_scheduler_py
