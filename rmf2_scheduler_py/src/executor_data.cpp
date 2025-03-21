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

#include "nlohmann/json.hpp"
#include "pybind11_json/pybind11_json.hpp"
#include "rmf2_scheduler_py/executor_data.hpp"
#include "rmf2_scheduler/executor_data.hpp"

namespace rmf2_scheduler_py
{

void init_executor_data_py(py::module & m)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  py::class_<ExecutorData>(
    m,
    "ExecutorData"
  )
  .def(py::init<>())
  .def(
    "set_data_as_string",
    &ExecutorData::set_data_as_string
  )
  .def(
    "set_data_as_json",
    &ExecutorData::set_data_as_json
  )
  .def(
    "get_data_as_string",
    &ExecutorData::get_data_as_string
  )
  .def(
    "get_data_as_json",
    &ExecutorData::get_data_as_json
  )
  ;
}

}  // namespace rmf2_scheduler_py
