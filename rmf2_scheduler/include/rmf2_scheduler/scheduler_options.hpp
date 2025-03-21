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

#ifndef RMF2_SCHEDULER__SCHEDULER_OPTIONS_HPP_
#define RMF2_SCHEDULER__SCHEDULER_OPTIONS_HPP_

#include "rmf2_scheduler/macros.hpp"
#include "rmf2_scheduler/data/duration.hpp"

namespace rmf2_scheduler
{

class SchedulerOptions : public std::enable_shared_from_this<SchedulerOptions>
{
public:
  RS_SMART_PTR_DEFINITIONS(SchedulerOptions)

  SchedulerOptions();
  virtual ~SchedulerOptions();

  SchedulerOptions::Ptr runtime_tick_period(const data::Duration & duration);

  const data::Duration & runtime_tick_period() const;

  SchedulerOptions::Ptr static_cache_period(const data::Duration & duration);

  const data::Duration & static_cache_period() const;

  SchedulerOptions::Ptr allow_past_tasks_duration(const data::Duration & duration);

  const data::Duration & allow_past_tasks_duration() const;

private:
  RS_DISABLE_COPY(SchedulerOptions)

  data::Duration runtime_tick_period_;
  data::Duration static_cache_period_;
  data::Duration allow_past_tasks_duration_;
};

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__SCHEDULER_OPTIONS_HPP_
