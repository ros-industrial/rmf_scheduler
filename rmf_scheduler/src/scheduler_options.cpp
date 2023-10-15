// Copyright 2023 ROS Industrial Consortium Asia Pacific
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

#include "rmf_scheduler/scheduler_options.hpp"

namespace rmf_scheduler
{

SchedulerOptions::SchedulerOptions()
{
}

SchedulerOptions & SchedulerOptions::tick_period(double sec)
{
  tick_period_ = sec;
  return *this;
}

SchedulerOptions & SchedulerOptions::allow_past_events_duration(double sec)
{
  allow_past_events_duration_ = sec;
  return *this;
}

SchedulerOptions & SchedulerOptions::series_max_expandable_duration(double sec)
{
  series_max_expandable_duration_ = sec;
  return *this;
}

SchedulerOptions & SchedulerOptions::expand_series_automatically(bool flag)
{
  expand_series_ = flag;
  return *this;
}

SchedulerOptions & SchedulerOptions::estimate_timeout(double sec)
{
  estimate_timeout_ = sec;
  return *this;
}

}  // namespace rmf_scheduler
