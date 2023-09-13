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

#ifndef RMF_SCHEDULER_PLUGINS__ROBOT_TASK_BUILDER_HPP_
#define RMF_SCHEDULER_PLUGINS__ROBOT_TASK_BUILDER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rmf_scheduler/task/builder_interface.hpp"

namespace rmf_scheduler_plugins
{

class RobotTaskBuilder : public rmf_scheduler::task::BuilderInterface
{
public:
  RobotTaskBuilder();
  virtual ~RobotTaskBuilder();

  void init(const std::shared_ptr<void> & node) override;

  void build_task(
    const nlohmann::json & event_details,
    nlohmann::json & task_details) override;
};

}  // namespace rmf_scheduler_plugins

#endif  // RMF_SCHEDULER_PLUGINS__ROBOT_TASK_BUILDER_HPP_
