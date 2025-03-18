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

#include "rmf2_scheduler_py/cache/schedule_cache.hpp"
#include "rmf2_scheduler/cache/schedule_cache.hpp"

namespace rmf2_scheduler_py
{

namespace cache
{

void init_schedule_cache_py(py::module & m)
{
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)

  py::module m_cache = m.def_submodule("cache");

  py::class_<
    ScheduleCache,
    ScheduleCache::Ptr
  >(
    m_cache,
    "ScheduleCache",
    R"(
    Cache for the schedule
    )"
  )
  .def(py::init<>())
  .def(
    "get_event",
    &ScheduleCache::get_event
  )
  .def(
    "get_all_events",
    &ScheduleCache::get_all_events
  )
  .def(
    "has_event",
    &ScheduleCache::has_event
  )
  .def(
    "lookup_events",
    &ScheduleCache::lookup_events
  )
  .def(
    "get_task",
    &ScheduleCache::get_task
  )
  .def(
    "get_all_tasks",
    &ScheduleCache::get_all_tasks
  )
  .def(
    "has_task",
    &ScheduleCache::has_task
  )
  .def(
    "lookup_tasks",
    &ScheduleCache::lookup_tasks
  )
  .def(
    "get_process",
    &ScheduleCache::get_process
  )
  .def(
    "get_all_processes",
    &ScheduleCache::get_all_processes
  )
  .def(
    "has_process",
    &ScheduleCache::has_process
  )
  .def(
    "clone",
    &ScheduleCache::clone
  )
  ;
}

}  // namespace cache

}  // namespace rmf2_scheduler_py
