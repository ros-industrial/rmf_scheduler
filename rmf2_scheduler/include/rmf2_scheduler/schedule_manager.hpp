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

#ifndef RMF2_SCHEDULER__SCHEDULE_MANAGER_HPP_
#define RMF2_SCHEDULER__SCHEDULE_MANAGER_HPP_

#include "rmf2_scheduler/storage/schedule_stream.hpp"
#include "rmf2_scheduler/handler/schedule_handler.hpp"

namespace rmf2_scheduler
{

class ScheduleManager
{
public:
  ScheduleManager(
    storage::ScheduleStream::Ptr schedule_stream
  );

  virtual ~ScheduleManager();

  ScheduleHandler::Ptr create_crud_handler();

  // ScheduleHandler::Ptr create_optimization_handler();

  // ScheduleHandler::Ptr create_execution_handler();

private:
  storage::ScheduleStream::Ptr stream_;
};

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__SCHEDULE_MANAGER_HPP_
