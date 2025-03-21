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

#ifndef RMF2_SCHEDULER__STORAGE__SCHEDULE_STREAM_HPP_
#define RMF2_SCHEDULER__STORAGE__SCHEDULE_STREAM_HPP_

#include <string>
#include <vector>

#include "rmf2_scheduler/cache/schedule_cache.hpp"
#include "rmf2_scheduler/data/schedule_change_record.hpp"
#include "rmf2_scheduler/data/time_window.hpp"
#include "rmf2_scheduler/time_window_lookup.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace storage
{

class ScheduleStream
{
public:
  RS_SMART_PTR_ALIASES_ONLY(ScheduleStream)

  ScheduleStream() = default;
  virtual ~ScheduleStream() = default;

  virtual bool read_schedule(
    cache::ScheduleCache::Ptr cache,
    const data::TimeWindow & time_window,
    std::string & error
  ) = 0;

  virtual bool write_schedule(
    cache::ScheduleCache::ConstPtr cache,
    const data::TimeWindow & time_window,
    std::string & error
  ) = 0;

  virtual bool write_schedule(
    cache::ScheduleCache::ConstPtr cache,
    const std::vector<data::ScheduleChangeRecord> & records,
    std::string & error
  ) = 0;

  virtual bool refresh_tasks(
    cache::ScheduleCache::Ptr cache,
    const std::vector<std::string> & ids,
    std::string & error
  ) = 0;

  static ScheduleStream::Ptr create_default(
    const std::string & url
  );
};

}  // namespace storage
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__STORAGE__SCHEDULE_STREAM_HPP_
