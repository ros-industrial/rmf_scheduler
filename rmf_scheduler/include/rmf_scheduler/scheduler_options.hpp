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

#include <string>
#include <unordered_map>

namespace rmf_scheduler
{

/// Encapsulation of options for scheduler initialization.
class SchedulerOptions
{
  friend class Scheduler;

public:
  using DynamicChargerMap =
    std::unordered_map<
    std::string, std::unordered_map<std::string, std::string>
    >;
  using FixedChargerMap = std::unordered_map<std::string, std::string>;
  SchedulerOptions();

  virtual ~SchedulerOptions() = default;

  SchedulerOptions & tick_period(double sec);
  SchedulerOptions & allow_past_events_duration(double sec);
  SchedulerOptions & series_max_expandable_duration(double sec);
  SchedulerOptions & expand_series_automatically(bool);
  SchedulerOptions & estimate_timeout(double sec);
  SchedulerOptions & enable_optimization(bool);
  SchedulerOptions & optimization_window(const std::string &);
  SchedulerOptions & optimization_window_timezone(const std::string &);
  SchedulerOptions & enable_local_caching(bool);
  SchedulerOptions & cache_dir(const std::string &);
  SchedulerOptions & cache_keep_last(size_t);
  SchedulerOptions & dynamic_charger_map(const DynamicChargerMap &);
  SchedulerOptions & fixed_charger_map(const FixedChargerMap &);

private:
  // TODO(Briancbn): use context here
  double tick_period_ = 5 * 60;  // 5min
  double allow_past_events_duration_ = 5 * 60;  // 5min
  double series_max_expandable_duration_ = 2 * 30 * 24 * 60 * 60;  // 2months
  bool expand_series_ = true;
  double estimate_timeout_ = 2.0;  // 2s

  bool enable_optimization_ = false;
  std::string optimization_window_ = "";
  std::string optimization_window_timezone_ = "Asia/Singapore";

  bool enable_local_caching_ = false;
  std::string cache_dir_ = ".";
  size_t cache_keep_last_ = 5;

  DynamicChargerMap dynamic_charger_map_;
  FixedChargerMap fixed_charger_map_;
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__SCHEDULER_OPTIONS_HPP_
