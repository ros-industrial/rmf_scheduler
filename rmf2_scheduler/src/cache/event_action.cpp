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

#include "rmf2_scheduler/cache/event_action.hpp"

namespace rmf2_scheduler
{

namespace cache
{

EventAction::EventAction(
  const std::string & type,
  const ActionPayload & payload
)
: Action(type, payload)
{
}

EventAction::~EventAction()
{
}

bool EventAction::validate(
  const ScheduleCache::Ptr & cache,
  std::string & error
)
{
  if (!on_validate_prepare(error)) {
    return false;
  }

  // Check if action type is valid
  auto event_action_type = data_.type;
  if (event_action_type.find(data::action_type::EVENT_PREFIX) != 0) {
    std::ostringstream oss;
    oss << "EVENT action invalid [" << event_action_type << "].";
    error = oss.str();
    return false;
  }

  bool result = false;

  // Create task handler
  ScheduleCache::TaskRestricted token;
  auto task_handler = cache->make_task_handler(token);

  if (event_action_type == data::action_type::EVENT_ADD) {
    // EVENT_ADD
    result = _validate_event_add(task_handler, error);
  } else if (event_action_type == data::action_type::EVENT_UPDATE) {
    // EVENT_UPDATE
    result = _validate_event_update(task_handler, error);
  } else if (event_action_type == data::action_type::EVENT_DELETE) {
    // EVENT_DELETE
    result = _validate_event_delete(task_handler, error);
  } else {
    std::ostringstream oss;
    oss << "Undefined EVENT action [" << event_action_type << "].";
    error = oss.str();
    return false;
  }

  if (result) {
    task_handler_ = task_handler;
    on_validate_complete();
  }

  return result;
}

void EventAction::apply()
{
  on_apply_prepare();

  auto event_action_type = data_.type;

  if (event_action_type == data::action_type::EVENT_ADD) {
    // EVENT_ADD
    task_handler_->emplace(data_.event);
    record_.add("event", {{data_.event->id, "add"}});
  } else if (event_action_type == data::action_type::EVENT_UPDATE) {
    // EVENT_UPDATE
    task_handler_->replace(event_itr_, data_.event);
    record_.add("event", {{data_.event->id, "update"}});
  } else if (event_action_type == data::action_type::EVENT_DELETE) {
    // EVENT_DELETE
    task_handler_->erase(event_itr_);
    record_.add("event", {{*data_.id, "delete"}});
  } else {
    assert(false);
  }

  on_apply_complete();
}

bool EventAction::_validate_event_add(
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  auto event = data_.event;
  if (!event) {
    error = "EVENT_ADD failed, no event found in payload.";
    return false;
  }

  TaskHandler::TaskConstIterator itr;
  if (task_handler->find(event->id, itr)) {
    error = "EVENT_ADD failed, event [" + event->id + "] already exists.";
    return false;
  }

  // Don't allow setting series and process ID through EVENT_ADD
  if (event->series_id.has_value()) {
    error = "EVENT_ADD warning, event [" + event->id + "]'s series_id is ignored.";
    event->series_id.reset();
  }

  if (event->process_id.has_value()) {
    if (!error.empty()) {
      error += '\n';
    }
    error += "EVENT_ADD warning, event [" + event->id + "]'s process_id is ignored.";
    event->process_id.reset();
  }

  return true;
}

bool EventAction::_validate_event_update(
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  auto event = data_.event;
  if (!event) {
    error = "EVENT_UPDATE failed, no event found in payload.";
    return false;
  }

  bool event_exist = task_handler->find(event->id, event_itr_);

  if (!event_exist) {
    error = "EVENT_UPDATE failed, event [" + event->id + "] doesn't exist.";
    return false;
  }

  // Don't allow setting series and process ID through EVENT_UPDATE
  auto old_event = event_itr_->second;
  if (event->series_id != old_event->series_id) {
    error = "EVENT_UPDATE warning, event [" + event->id + "]'s new series_id is ignored.";
    event->series_id = old_event->series_id;
  }

  if (event->process_id != old_event->process_id) {
    error = "EVENT_UPDATE warning, event [" + event->id + "]'s new process_id is ignored.";
    event->process_id = old_event->process_id;
  }

  return true;
}

bool EventAction::_validate_event_delete(
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  if (!data_.id.has_value()) {
    error = "EVENT_DELETE failed, no ID found in payload.";
    return false;
  }

  auto id = data_.id.value();

  bool event_exist = task_handler->find(id, event_itr_);

  if (!event_exist) {
    error = "EVENT_DELETE failed, event [" + id + "] doesn't exist.";
    return false;
  }

  // Don't allow removing events that are part of a process or series EVENT_DELETE
  auto event = event_itr_->second;
  if (event->series_id.has_value()) {
    error = "EVENT_DELETE failed, event [" +
      event->id + "] is part of a series, please detach it first.";
    return false;
  }

  if (event->process_id.has_value()) {
    error = "EVENT_DELETE failed, event [" +
      event->id + "] is part of a process, please detach it first.";
    return false;
  }

  return true;
}

}  // namespace cache
}  // namespace rmf2_scheduler
