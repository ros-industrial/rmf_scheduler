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

#ifndef RMF_SCHEDULER__TASK__BUILDER_INTERFACE_HPP_
#define RMF_SCHEDULER__TASK__BUILDER_INTERFACE_HPP_

#include <memory>

#include "rmf_scheduler/task/task_plugin.hpp"
#include "nlohmann/json.hpp"

namespace rmf_scheduler
{

namespace task
{

class BuilderInterface : public TaskPluginBase
{
public:
  virtual ~BuilderInterface() {}

  virtual void init(const std::shared_ptr<void> & node) = 0;

  virtual void build_task(
    const nlohmann::json & event_details,
    nlohmann::json & task_details) = 0;
};

}  // namespace task

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__TASK__BUILDER_INTERFACE_HPP_
