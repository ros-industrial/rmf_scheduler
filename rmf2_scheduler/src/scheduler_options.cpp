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

#include "rmf2_scheduler/scheduler_options.hpp"

namespace rmf2_scheduler
{

SchedulerOptions::SchedulerOptions()
: runtime_tick_period_(std::chrono::minutes(5)),  // 5 min for the runtime ticking
  static_cache_period_(std::chrono::minutes(60 * 24 * 7)),  // 1 week of static cache
  allow_past_tasks_duration_(std::chrono::minutes(1))  // 1 min for past tasks allowed
{
}

SchedulerOptions::~SchedulerOptions()
{
}

SchedulerOptions::Ptr SchedulerOptions::runtime_tick_period(const data::Duration & duration)
{
  runtime_tick_period_ = duration;
  return shared_from_this();
}

const data::Duration & SchedulerOptions::runtime_tick_period() const
{
  return runtime_tick_period_;
}

SchedulerOptions::Ptr SchedulerOptions::static_cache_period(const data::Duration & duration)
{
  static_cache_period_ = duration;
  return shared_from_this();
}

const data::Duration & SchedulerOptions::static_cache_period() const
{
  return static_cache_period_;
}

SchedulerOptions::Ptr SchedulerOptions::allow_past_tasks_duration(const data::Duration & duration)
{
  allow_past_tasks_duration_ = duration;
  return shared_from_this();
}

const data::Duration & SchedulerOptions::allow_past_tasks_duration() const
{
  return allow_past_tasks_duration_;
}
}  // namespace rmf2_scheduler
