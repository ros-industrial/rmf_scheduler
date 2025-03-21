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

#ifndef RMF2_SCHEDULER__PROCESS_EXECUTOR_HPP_
#define RMF2_SCHEDULER__PROCESS_EXECUTOR_HPP_

#include <string>
#include <vector>

#include "rmf2_scheduler/data/task.hpp"
#include "rmf2_scheduler/data/process.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

class ProcessExecutor
{
public:
  RS_SMART_PTR_DEFINITIONS(ProcessExecutor)

  ProcessExecutor() = default;
  virtual ~ProcessExecutor() {}

  virtual bool run_async(
    const data::Process::ConstPtr &,
    const std::vector<data::Task::ConstPtr> &,
    std::string & error
  ) = 0;

private:
  RS_DISABLE_COPY(ProcessExecutor)
};

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__PROCESS_EXECUTOR_HPP_
