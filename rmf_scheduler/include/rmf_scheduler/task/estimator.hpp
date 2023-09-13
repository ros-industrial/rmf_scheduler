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

#ifndef RMF_SCHEDULER__TASK__ESTIMATOR_HPP_
#define RMF_SCHEDULER__TASK__ESTIMATOR_HPP_

#include <future>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rmf_scheduler/task/task_plugin.hpp"
#include "rmf_scheduler/task/estimate_interface.hpp"
#include "rmf_scheduler/data/event.hpp"

namespace rmf_scheduler
{

namespace task
{

class Estimator : public TaskPluginManager
{
public:
  Estimator();

  virtual ~Estimator();

  void load_plugin(
    const std::shared_ptr<void> & node,
    const std::string & name,
    const std::string & interface,
    const std::vector<std::string> & supported_tasks) override;

  void unload_plugin(
    const std::string & name) override;

  /// Estimate the events based on the existing estimated states
  void estimate(
    std::unordered_map<std::string, data::Event> & events_to_add,
    double timeout);

  bool validate_estimate_states(
    const std::string & robot_name,
    uint64_t start_time,
    uint64_t end_time) const;

  void add_estimate_states(
    const std::string & robot_name,
    uint64_t time,
    const task::EstimateStates & estimated_state);

  void delete_estimate_states(
    const std::string & robot_name,
    uint64_t start_time,
    uint64_t end_time);

private:
  std::unordered_map<std::string,
    std::multimap<uint64_t, task::EstimateStates>> robot_estimate_states_;
};

}  // namespace task

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__TASK__ESTIMATOR_HPP_
