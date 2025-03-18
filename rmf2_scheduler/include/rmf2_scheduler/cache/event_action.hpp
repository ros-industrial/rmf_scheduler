// Copyright 2024 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__CACHE__EVENT_ACTION_HPP_
#define RMF2_SCHEDULER__CACHE__EVENT_ACTION_HPP_

#include <memory>
#include <string>

#include "rmf2_scheduler/cache/action.hpp"
#include "rmf2_scheduler/cache/schedule_cache.hpp"
#include "rmf2_scheduler/cache/task_handler.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace cache
{

class EventAction : public Action
{
public:
  RS_SMART_PTR_DEFINITIONS(EventAction)

  explicit EventAction(
    const std::string & type,
    const ActionPayload & payload
  );

  virtual ~EventAction();

  bool validate(
    const ScheduleCache::Ptr & cache,
    std::string & error
  ) override;

  void apply() override;

private:
  RS_DISABLE_COPY(EventAction)

  bool _validate_event_add(
    const TaskHandler::ConstPtr & task_handler,
    std::string & error
  );

  bool _validate_event_update(
    const TaskHandler::ConstPtr & task_handler,
    std::string & error
  );

  bool _validate_event_delete(
    const TaskHandler::ConstPtr & task_handler,
    std::string & error
  );

  TaskHandler::Ptr task_handler_;
  TaskHandler::TaskConstIterator event_itr_;
};

}  // namespace cache
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__CACHE__EVENT_ACTION_HPP_
