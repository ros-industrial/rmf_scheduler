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

#ifndef RMF_SCHEDULER__SCHEDULER_OPTIONS_HPP_
#define RMF_SCHEDULER__SCHEDULER_OPTIONS_HPP_

namespace rmf_scheduler
{

/// Encapsulation of options for scheduler initialization.
class SchedulerOptions
{
  friend class Scheduler;

public:
  SchedulerOptions();

  virtual ~SchedulerOptions() = default;

  SchedulerOptions & tick_period(double sec);
  SchedulerOptions & allow_past_events_duration(double sec);
  SchedulerOptions & series_max_expandable_duration(double sec);
  SchedulerOptions & expand_series_automatically(bool);

private:
  // TODO(Briancbn): use context here
  double tick_period_ = 5 * 60;  // 5min
  double allow_past_events_duration_ = 5 * 60;  // 5min
  double series_max_expandable_duration_ = 2 * 30 * 24 * 60 * 60;  // 2months
  double expand_series_ = true;
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__SCHEDULER_OPTIONS_HPP_
