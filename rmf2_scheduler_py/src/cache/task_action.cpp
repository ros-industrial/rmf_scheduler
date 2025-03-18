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

#include "rmf2_scheduler_py/cache/task_action.hpp"
#include "rmf2_scheduler/cache/task_action.hpp"

namespace rmf2_scheduler_py
{

namespace cache
{

void init_task_action_py(py::module & m)
{
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)

  py::module m_cache = m.def_submodule("cache");

  py::class_<
    TaskAction,
    TaskAction::Ptr,
    Action
  >(
    m_cache,
    "TaskAction",
    R"(
    Change action for Task
    )"
  )
  .def(
    py::init<const std::string &, const ActionPayload &>(),
    py::arg("type"),
    py::arg("payload")
  )
  .def(
    "validate",
    [](
      TaskAction & self,
      const ScheduleCache::Ptr & cache
    ) {
      std::string error;
      bool result = self.validate(cache, error);
      return py::make_tuple(result, error);
    }
  )
  .def(
    "apply",
    &TaskAction::apply
  )
  ;
}

}  // namespace cache

}  // namespace rmf2_scheduler_py
