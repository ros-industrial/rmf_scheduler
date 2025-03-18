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

#include "rmf2_scheduler_py/scheduler_options.hpp"
#include "rmf2_scheduler/scheduler_options.hpp"

namespace rmf2_scheduler_py
{

void init_scheduler_options_py(py::module & m)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  py::class_<
    SchedulerOptions,
    SchedulerOptions::Ptr
  >(
    m,
    "SchedulerOptions",
    R"(
    SchedulerOptions Class
    )"
  )
  .def(py::init<>())
  .def(
    "runtime_tick_period",
    py::overload_cast<const data::Duration &>(
      &SchedulerOptions::runtime_tick_period
    )
  )
  .def(
    "runtime_tick_period",
    py::overload_cast<>(
      &SchedulerOptions::runtime_tick_period,
      py::const_
    )
  )
  .def(
    "static_cache_period",
    py::overload_cast<const data::Duration &>(
      &SchedulerOptions::static_cache_period
    )
  )
  .def(
    "static_cache_period",
    py::overload_cast<>(
      &SchedulerOptions::static_cache_period,
      py::const_
    )
  )
  ;
}

}  // namespace rmf2_scheduler_py
