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

#include "rmf_scheduler_plugins/robot_task_builder.hpp"
#include "rmf_scheduler_plugins/utils.hpp"

namespace rmf_scheduler_plugins
{

RobotTaskBuilder::RobotTaskBuilder()
{
}

RobotTaskBuilder::~RobotTaskBuilder()
{
}


void RobotTaskBuilder::init(const std::shared_ptr<void> &)
{
}


void RobotTaskBuilder::build_task(
  const nlohmann::json & event_details,
  nlohmann::json & task_details)
{
  if (event_details.contains("request")) {
    task_details["request"] = event_details["request"];
    task_details["robot"] =
      to_slug(event_details["robot"].get<std::string>());
    task_details["fleet"] =
      to_slug(event_details["fleet"].get<std::string>());
  } else {
    throw rmf_scheduler::exception::InvalidTaskSchemaException(
            "Event details doesn't contains request");
  }
}

}  // namespace rmf_scheduler_plugins

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rmf_scheduler_plugins::RobotTaskBuilder, rmf_scheduler::task::BuilderInterface)
