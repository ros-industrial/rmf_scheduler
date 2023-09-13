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

#ifndef RMF_SCHEDULER__TASK__BUILDER_HPP_
#define RMF_SCHEDULER__TASK__BUILDER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rmf_scheduler/task/task_plugin.hpp"
#include "rmf_scheduler/task/builder_interface.hpp"
#include "rmf_scheduler/data/event.hpp"

namespace rmf_scheduler
{

namespace task
{

class Builder : public TaskPluginManager
{
public:
  Builder();

  virtual ~Builder();

  void load_plugin(
    const std::shared_ptr<void> & node,
    const std::string & name,
    const std::string & interface,
    const std::vector<std::string> & supported_tasks) override;

  void unload_plugin(
    const std::string & name) override;

  void build_task(data::Event & event);
};

}  // namespace task

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__TASK__BUILDER_HPP_
