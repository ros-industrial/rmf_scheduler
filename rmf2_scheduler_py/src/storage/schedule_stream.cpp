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

#include "rmf2_scheduler_py/storage/schedule_stream.hpp"
#include "rmf2_scheduler/storage/schedule_stream.hpp"

namespace rmf2_scheduler_py
{

namespace storage
{

void init_schedule_stream_py(py::module & m)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::storage;  // NOLINT(build/namespaces)

  py::module m_storage = m.def_submodule("storage");

  py::class_<
    ScheduleStream,
    ScheduleStream::Ptr
  >(
    m_storage,
    "ScheduleStream",
    R"(
    Stream for the schedule
    )"
  )
  .def(
    "read_schedule",
    [](
      ScheduleStream & self,
      cache::ScheduleCache::Ptr schedule_cache,
      const data::TimeWindow & time_window
    ) {
      std::string error;
      bool result = self.read_schedule(schedule_cache, time_window, error);
      return py::make_tuple(result, error);
    }
  )
  .def(
    "write_schedule",
    [](
      ScheduleStream & self,
      cache::ScheduleCache::ConstPtr schedule_cache,
      const data::TimeWindow & time_window
    ) {
      std::string error;
      bool result = self.write_schedule(schedule_cache, time_window, error);
      return py::make_tuple(result, error);
    }
  )
  .def(
    "write_schedule",
    [](
      ScheduleStream & self,
      cache::ScheduleCache::ConstPtr schedule_cache,
      const std::vector<data::ScheduleChangeRecord> & records
    ) {
      std::string error;
      bool result = self.write_schedule(schedule_cache, records, error);
      return py::make_tuple(result, error);
    }
  )
  .def_static(
    "create_default",
    &ScheduleStream::create_default
  )
  ;
}

}  // namespace storage

}  // namespace rmf2_scheduler_py
