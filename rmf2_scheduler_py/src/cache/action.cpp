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

#include "rmf2_scheduler_py/cache/action.hpp"
#include "rmf2_scheduler/cache/schedule_cache.hpp"
#include "rmf2_scheduler/cache/action.hpp"

namespace rmf2_scheduler_py
{

namespace cache
{

void init_action_py(py::module & m)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)

  py::module m_cache = m.def_submodule("cache");

  // ActionPayload
  py::class_<ActionPayload>(
    m_cache,
    "ActionPayload",
    R"(
    Payload for the cache change action
    )"
  )
  .def(py::init<>())
  .def(
    "id",
    &ActionPayload::id
  )
  .def(
    "event",
    &ActionPayload::event
  )
  .def(
    "task",
    &ActionPayload::task
  )
  .def(
    "process",
    &ActionPayload::process
  )
  .def(
    "node_id",
    &ActionPayload::node_id
  )
  .def(
    "source_id",
    &ActionPayload::source_id
  )
  .def(
    "destination_id",
    &ActionPayload::destination_id
  )
  .def(
    "edge_type",
    &ActionPayload::edge_type
  )
  .def(
    "data",
    &ActionPayload::data
  )
  ;

  // Action
  py::class_<
    Action,
    Action::Ptr
  >(
    m_cache,
    "Action",
    R"(
    Cache change action for the schedule
    )"
  )
  .def(
    "data",
    &Action::data
  )
  .def(
    "record",
    &Action::record
  )
  .def(
    "validate",
    [](
      Action & self,
      const ScheduleCache::Ptr & cache
    ) {
      std::string error;
      bool result = self.validate(cache, error);
      return py::make_tuple(result, error);
    }
  )
  .def(
    "apply",
    &Action::apply
  )
  .def_static(
    "create",
    py::overload_cast<const std::string &, const ActionPayload &>(
      &Action::create
    )
  )
  .def_static(
    "create",
    py::overload_cast<const data::ScheduleAction &>(
      &Action::create
    )
  )
  .def(
    "is_valid",
    &Action::is_valid
  )
  .def(
    "is_applied",
    &Action::is_applied
  )
  ;
}

}  // namespace cache

}  // namespace rmf2_scheduler_py
