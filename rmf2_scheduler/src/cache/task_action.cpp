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

#include "rmf2_scheduler/cache/task_action.hpp"

namespace rmf2_scheduler
{

namespace cache
{

TaskAction::TaskAction(
  const std::string & type,
  const ActionPayload & payload
)
: Action(type, payload)
{
}

TaskAction::~TaskAction()
{
}

bool TaskAction::validate(
  const ScheduleCache::Ptr & cache,
  std::string & error
)
{
  if (!on_validate_prepare(error)) {
    return false;
  }

  // Check if action type is valid
  auto task_action_type = data_.type;
  if (task_action_type.find(data::action_type::TASK_PREFIX) != 0) {
    std::ostringstream oss;
    oss << "TASK action invalid [" << task_action_type << "].";
    error = oss.str();
    return false;
  }

  bool result = false;

  // Create task handler
  ScheduleCache::TaskRestricted token;
  auto task_handler = cache->make_task_handler(token);

  if (task_action_type == data::action_type::TASK_ADD) {
    // TASK_ADD
    result = _validate_task_add(task_handler, error);
  } else if (task_action_type == data::action_type::TASK_UPDATE) {
    // TASK_UPDATE
    result = _validate_task_update(task_handler, error);
  } else if (task_action_type == data::action_type::TASK_DELETE) {
    // TASK_DELETE
    result = _validate_task_delete(task_handler, error);
  } else {
    std::ostringstream oss;
    oss << "Undefined TASK action [" << task_action_type << "].";
    error = oss.str();
    return false;
  }

  if (result) {
    task_handler_ = task_handler;
    on_validate_complete();
  }

  return result;
}

void TaskAction::apply()
{
  on_apply_prepare();

  auto task_action_type = data_.type;

  if (task_action_type == data::action_type::TASK_ADD) {
    // TASK_ADD
    task_handler_->emplace(data_.task);
    record_.add("task", {{data_.task->id, "add"}});
  } else if (task_action_type == data::action_type::TASK_UPDATE) {
    // TASK_UPDATE
    task_handler_->replace(task_itr_, data_.task);
    record_.add("task", {{data_.task->id, "update"}});
  } else if (task_action_type == data::action_type::TASK_DELETE) {
    // TASK_DELETE
    task_handler_->erase(task_itr_);
    record_.add("task", {{*data_.id, "delete"}});
  } else {
    assert(false);
  }

  on_apply_complete();
}

bool TaskAction::_validate_task_add(
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  auto task = data_.task;
  if (!task) {
    error = "TASK_ADD failed, no task found in payload.";
    return false;
  }

  TaskHandler::TaskConstIterator itr;
  if (task_handler->find(task->id, itr)) {
    error = "TASK_ADD failed, task [" + task->id + "] already exists.";
    return false;
  }

  // Don't allow setting series and process ID through TASK_ADD
  if (task->series_id.has_value()) {
    error = "TASK_ADD warning, task [" + task->id + "]'s series_id is ignored.";
    task->series_id.reset();
  }

  if (task->process_id.has_value()) {
    if (!error.empty()) {
      error += '\n';
    }
    error += "TASK_ADD warning, task [" + task->id + "]'s process_id is ignored.";
    task->process_id.reset();
  }

  return true;
}

bool TaskAction::_validate_task_update(
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  auto task = data_.task;
  if (!task) {
    error = "TASK_UPDATE failed, no task found in payload.";
    return false;
  }

  bool task_exist = task_handler->find(task->id, task_itr_);

  if (!task_exist) {
    error = "TASK_UPDATE failed, task [" + task->id + "] doesn't exist.";
    return false;
  }

  // Don't allow setting series and process ID through TASK_UPDATE
  auto old_task = task_itr_->second;
  if (task->series_id != old_task->series_id) {
    error = "TASK_UPDATE warning, task [" + task->id + "]'s new series_id is ignored.";
    task->series_id = old_task->series_id;
  }

  if (task->process_id != old_task->process_id) {
    error = "TASK_UPDATE warning, task [" + task->id + "]'s new process_id is ignored.";
    task->process_id = old_task->process_id;
  }

  return true;
}

bool TaskAction::_validate_task_delete(
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  if (!data_.id.has_value()) {
    error = "TASK_DELETE failed, no ID found in payload.";
    return false;
  }

  auto id = data_.id.value();

  bool task_exist = task_handler->find(id, task_itr_);

  if (!task_exist) {
    error = "TASK_DELETE failed, task [" + id + "] doesn't exist.";
    return false;
  }

  // Don't allow removing events that are part of a process or series EVENT_DELETE
  auto task = task_itr_->second;
  if (task->series_id.has_value()) {
    error = "TASK_DELETE failed, task [" +
      task->id + "] is part of a series, please detach it first.";
    return false;
  }

  if (task->process_id.has_value()) {
    error = "TASK_DELETE failed, task [" +
      task->id + "] is part of a process, please detach it first.";
    return false;
  }

  return true;
}

}  // namespace cache
}  // namespace rmf2_scheduler
