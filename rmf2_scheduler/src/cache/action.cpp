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

#include "rmf2_scheduler/cache/action.hpp"
#include "rmf2_scheduler/cache/event_action.hpp"
#include "rmf2_scheduler/cache/task_action.hpp"
#include "rmf2_scheduler/cache/process_action.hpp"

namespace rmf2_scheduler
{

namespace cache
{

std::shared_ptr<Action> Action::create(
  const std::string & type,
  const ActionPayload & payload
)
{
  if (type.find(data::action_type::EVENT_PREFIX) == 0) {
    return EventAction::make_shared(type, payload);
  } else if (type.find(data::action_type::TASK_PREFIX) == 0) {
    return TaskAction::make_shared(type, payload);
  } else if (type.find(data::action_type::PROCESS_PREFIX) == 0) {
    return ProcessAction::make_shared(type, payload);
  } else {
    throw std::runtime_error("Invalid action type");
  }
}

std::shared_ptr<Action> Action::create(
  const data::ScheduleAction & data
)
{
  ActionPayload payload;
  payload.data_ = data;
  return create(data.type, payload);
}

}  // namespace cache
}  // namespace rmf2_scheduler
