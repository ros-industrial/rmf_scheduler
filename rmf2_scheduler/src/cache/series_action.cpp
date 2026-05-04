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

#include "croncpp/croncpp.h"

#include "rmf2_scheduler/cache/series_action.hpp"
#include "rmf2_scheduler/data/occurrence.hpp"
#include "rmf2_scheduler/data/uuid.hpp"

namespace rmf2_scheduler
{

namespace cache
{

SeriesAction::SeriesAction(
  const std::string & type,
  const ActionPayload & payload
)
: Action(type, payload)
{
}

SeriesAction::~SeriesAction()
{
}

bool SeriesAction::validate(
  const ScheduleCache::Ptr & cache,
  std::string & error
)
{
  if (!on_validate_prepare(error)) {
    return false;
  }

  // Check if action type is valid
  auto series_action_type = data_.type;
  if (series_action_type.find(data::action_type::SERIES_PREFIX) != 0) {
    std::ostringstream oss;
    oss << "SERIES action invalid [" << series_action_type << "].";
    error = oss.str();
    return false;
  }

  bool result = false;

  // Create task handler
  auto series_handler = cache->make_series_handler(ScheduleCache::SeriesRestricted());
  auto task_handler = cache->make_task_handler(ScheduleCache::TaskRestricted());
  auto process_handler = cache->make_process_handler(ScheduleCache::ProcessRestricted());

  if (series_action_type == data::action_type::SERIES_ADD) {
    // SERIES ADD
    result = _validate_series_add(series_handler, task_handler, process_handler, error);
  } else if (series_action_type == data::action_type::SERIES_UPDATE) {
    // SERIES_UPDATE
    result = _validate_series_update(series_handler, error);
  } else if (series_action_type == data::action_type::SERIES_EXPAND_UNTIL) {
    // SERIES_EXPAND_UNTIL
    result = _validate_series_expand_until(series_handler, error);
  } else if (series_action_type == data::action_type::SERIES_UPDATE_CRON) {
    // SERIES_UPDATE_CRON
    result = _validate_series_update_cron(series_handler, error);
  } else if (series_action_type == data::action_type::SERIES_UPDATE_UNTIL) {
    // SERIES_UPDATE_UNTIL
    result = _validate_series_update_until(series_handler, error);
  } else if (series_action_type == data::action_type::SERIES_UPDATE_OCCURRENCE) {
    // SERIES_UPDATE_OCCURRENCE
    result = _validate_series_update_occurrence(series_handler, error);
  } else if (series_action_type == data::action_type::SERIES_UPDATE_OCCURRENCE_TIME) {
    // SERIES_UPDATE_OCCURRENCE_TIME
    result = _validate_series_update_occurrence_time(
      series_handler, task_handler, process_handler, error);
  } else if (series_action_type == data::action_type::SERIES_DELETE_OCCURRENCE) {
    // SERIES_DELETE_OCCURRENCE
    result = _validate_series_delete_occurrence(
      series_handler, task_handler, process_handler, error);
  } else if (series_action_type == data::action_type::SERIES_DELETE) {
    // SERIES_DELETE
    result = _validate_series_delete(series_handler, error);
  } else if (series_action_type == data::action_type::SERIES_DELETE_ALL) {
    // SERIES_DELETE_ALL
    result = _validate_series_delete_all(series_handler, error);
  } else {
    std::ostringstream oss;
    oss << "Undefined SERIES action [" << series_action_type << "].";
    error = oss.str();
    return false;
  }

  if (result) {
    task_handler_ = task_handler;
    process_handler_ = process_handler;
    series_handler_ = series_handler;
    on_validate_complete();
  }

  return result;
}

void SeriesAction::apply()
{
  on_apply_prepare();

  auto series_action_type = data_.type;

  if (series_action_type == data::action_type::SERIES_ADD) {
    // SERIES_ADD
    _apply_series_add();
  } else if (series_action_type == data::action_type::SERIES_UPDATE) {
    // SERIES_UPDATE
    assert(false);
  } else if (series_action_type == data::action_type::SERIES_EXPAND_UNTIL) {
    // SERIES_EXPAND_UNTIL
    _apply_series_expand_until();
  } else if (series_action_type == data::action_type::SERIES_UPDATE_CRON) {
    // SERIES_UPDATE_CRON
    _apply_series_update_cron();
  } else if (series_action_type == data::action_type::SERIES_UPDATE_UNTIL) {
    // SERIES_UPDATE_UNTIL
    _apply_series_update_until();
  } else if (series_action_type == data::action_type::SERIES_UPDATE_OCCURRENCE) {
    // SERIES_UPDATE_OCCURRENCE
    _apply_series_update_occurrence();
  } else if (series_action_type == data::action_type::SERIES_UPDATE_OCCURRENCE_TIME) {
    // SERIES_UPDATE_OCCURRENCE
    _apply_series_update_occurrence_time();
  } else if (series_action_type == data::action_type::SERIES_DELETE_OCCURRENCE) {
    // SERIES_DELETE_OCCURRENCE
    _apply_series_delete_occurrence();
  } else if (series_action_type == data::action_type::SERIES_DELETE) {
    // SERIES_DELETE
    _apply_series_delete_series();
  } else if (series_action_type == data::action_type::SERIES_DELETE_ALL) {
    // SERIES_DELETE_ALL
    _apply_series_delete_all();
  } else {
    assert(false);
  }

  on_apply_complete();
}

bool SeriesAction::_validate_series_add(
  const SeriesHandler::ConstPtr & series_handler,
  const TaskHandler::ConstPtr & task_handler,
  const ProcessHandler::ConstPtr & process_handler,
  std::string & error
)
{
  auto series = data_.series;
  if (!series) {
    error = "SERIES_ADD failed, no series found in payload.";
    return false;
  }

  // Check if series already exists
  SeriesHandler::SeriesConstIterator itr;
  if (series_handler->find(series->id(), itr)) {
    error = "SERIES_ADD failed, series [" + series->id() + "] already exists.";
    return false;
  }

  // Check if there are existing exceptions in the series
  if (series->exception_ids().size() != 0) {
    error = "SERIES_ADD failed, series [" + series->id() + "] should not have exceptions.";
    return false;
  }

  // Check if initial occurrence is provided.
  // Technically this is already checked during construction.
  if (series->occurrences().size() == 0) {
    error = "SERIES_ADD failed, series [" + series->id() + "] does not have an initial occurrence.";
    return false;
  }
  if (series->occurrences().size() > 1) {
    error = "SERIES_ADD failed, series [" + series->id() +
      "] there should only be one occurrence provided.";
    return false;
  }

  // Check if initial occurrence exists
  auto initial_occ = series->get_first_occurrence();
  if (series->type() == "task") {
    TaskHandler::TaskConstIterator itr;
    auto exists = task_handler->find(initial_occ.id, itr);
    if (!exists) {
      error = "SERIES_ADD failed, series [" + series->id() +
        "] initial occurrence, Task [" + initial_occ.id + "] does not exist.";
      return false;
    }
    // task exists but is part of a process
    if (itr->second->process_id.has_value()) {
      error = "SERIES_ADD failed, occurrence task [" + itr->first +
        "] is part of Process [" + itr->second->process_id.value() + "].";
      return false;
    }
  } else if (series->type() == "process") {
    ProcessHandler::ProcessConstIterator itr;
    auto exists = process_handler->find(initial_occ.id, itr);
    if (!exists) {
      error = "SERIES_ADD failed, series [" + series->id() +
        "] initial occurrence, Process [" + initial_occ.id + "] does not exist.";
      return false;
    }
    // Process action should have already validated the existence of the tasks
    // so we do not check again here.
  } else {
    error = "SERIES_ADD failed, series [" + series->id() +
      "] occurrence type [" + series->type() + "] does not exist.";
    return false;
  }
  return true;
}

void SeriesAction::_apply_series_add()
{
  auto series = data_.series;

  // get template initial occurrence
  auto first_occ = series->get_first_occurrence();
  const auto series_type = series->type();

  // TODO(Anyone) : Change this number to be confirgurable
  auto occ_to_add = series->expand_another(30);

  if (series_type == "task") {
    std::vector<data::ChangeAction> task_changes;
    task_changes.reserve(occ_to_add.size() + 1);

    // Add series id to the initial occurrence
    TaskHandler::TaskConstIterator task_itr;
    assert(task_handler_->find(first_occ.id, task_itr));
    task_itr->second->series_id = series->id();
    task_changes.push_back({task_itr->first, "update"});

    // Generate task occurrences and add them
    for (const auto & occ_itr : occ_to_add) {
      auto task_to_add = data::Task::make_shared(*task_itr->second);
      task_to_add->id = occ_itr.id;
      // In case the reference task is ongoing or completed
      task_to_add->status = "";
      task_to_add->start_time = occ_itr.time;
      task_to_add->series_id = series->id();
      task_handler_->emplace(task_to_add);
      task_changes.push_back({task_to_add->id, "add"});
    }
    record_.add("task", task_changes);
  } else if (series_type == "process") {
    // TODO(Anyone) We should find a more elegant way for
    // processes to generate their own tasks simply
    // by passing in a start time
    std::vector<data::ChangeAction> task_changes;
    std::vector<data::ChangeAction> process_changes;
    process_changes.reserve(occ_to_add.size() + 1);

    ProcessHandler::ProcessConstIterator process_itr;
    assert(process_handler_->find(first_occ.id, process_itr));

    // reserve task changes size
    task_changes.reserve((occ_to_add.size() + 1) * process_itr->second->graph.size());

    // Add series id to process
    process_itr->second->series_id = series->id();
    process_changes.push_back({process_itr->first, "update"});

    // Ensure initial tasks are set to first occurrence time
    process_itr->second->graph.for_each_node(
      [&first_occ, &task_changes, this](const data::Node::Ptr & node) {
        TaskHandler::TaskConstIterator task_itr;
        task_handler_->find(node->id(), task_itr);
        task_itr->second->start_time = first_occ.time;
        task_changes.push_back({node->id(), "update"});
      }
    );

    // Generate and add process and tasks occurrences
    for (const auto & occ_itr : occ_to_add) {
      // Create copy of process
      auto process_to_add = data::Process::make_shared(*process_itr->second);

      // Update process data
      process_to_add->series_id = series->id();
      process_to_add->id = occ_itr.id;

      // create and add tasks
      auto nodes = process_itr->second->graph.get_all_nodes();
      for (const auto & n : nodes) {
        // Find original task
        TaskHandler::TaskConstIterator task_itr;
        assert(task_handler_->find(n.first, task_itr));

        // generate and replace graph node with new id
        const auto new_id = data::gen_uuid();
        process_to_add->graph.update_node(task_itr->first, new_id);

        // Create and add task
        auto task_to_add = data::Task::make_shared(*task_itr->second);
        task_to_add->id = new_id;
        task_to_add->process_id = process_to_add->id;
        task_to_add->start_time = occ_itr.time;
        task_handler_->emplace(task_to_add);
        task_changes.push_back({task_to_add->id, "add"});
      }
      process_handler_->emplace(process_to_add);
      process_changes.push_back({process_to_add->id, "add"});
    }
    record_.add("task", task_changes);
    record_.add("process", process_changes);
  } else {
    assert(false);
  }

  // Add series
  series_handler_->emplace(series);
  record_.add("series", {{series->id(), "add"}});
}

// CURRENTLY NOT SUPPORTED
bool SeriesAction::_validate_series_update(
  const SeriesHandler::ConstPtr & series_handler,
  std::string & error)
{
  error = "SERIES_UPDATE currently not supported.";
  return false;
}

bool SeriesAction::_validate_series_expand_until(
  const SeriesHandler::ConstPtr & series_handler,
  std::string & error)
{
  if (!data_.id.has_value() || !data_.until.has_value()) {
    error = "SERIES_EXPAND_UNTIL failed, id and until must be set in payload.";
    return false;
  }

  // Check if series exists
  SeriesHandler::SeriesConstIterator itr;
  if (!series_handler->find(data_.id.value(), itr)) {
    error = "SERIES_EXPAND_UNTIL failed, series [" + data_.id.value() + "] does not exist.";
    return false;
  }

  // Check if expansion time is before first occurrence
  if (data_.until.value() < itr->second->get_first_occurrence().time) {
    error = "SERIES_EXPAND_UNTIL failed, provided time is before the frist occurrence.";
    return false;
  }

  // Check if expansion time is before last occurrence
  if (data_.until.value() < itr->second->get_last_occurrence().time) {
    error = "SERIES_EXPAND_UNTIL failed, provided time is before the last occurrence.";
    return false;
  }

  return true;
}

void SeriesAction::_apply_series_expand_until()
{
  SeriesHandler::SeriesConstIterator itr;
  assert(series_handler_->find(data_.id.value(), itr));
  data::Series::Ptr series_to_update = data::Series::make_shared(
    *itr->second
  );
  auto current_last_occ = itr->second->get_last_occurrence();
  // TODO(Anyone) : Change this number to be confirgurable
  auto occ_to_add = series_to_update->expand_until(data_.until.value());
  auto first_occ = series_to_update->get_first_occurrence();

  // for every series after current last occurrence,
  // generate task/ processes + task
  if (series_to_update->type() == "task") {
    std::vector<data::ChangeAction> task_changes;
    task_changes.reserve(occ_to_add.size());

    // Add series id to the initial occurrence
    TaskHandler::TaskConstIterator task_ref_itr;
    assert(task_handler_->find(first_occ.id, task_ref_itr));

    // Generate task occurrences and add them
    for (const auto & occ_itr : occ_to_add) {
      auto task_to_add = data::Task::make_shared(*task_ref_itr->second);
      task_to_add->id = occ_itr.id;
      // In case the reference task is ongoing or completed
      task_to_add->status = "";
      task_to_add->start_time = occ_itr.time;
      task_to_add->series_id = series_to_update->id();
      task_handler_->emplace(task_to_add);
      task_changes.push_back({task_to_add->id, "add"});
    }
    record_.add("task", task_changes);
  } else if (series_to_update->type() == "process") {
    // TODO(Anyone) We should find a more elegant way for
    // processes to generate their own tasks simply
    // by passing in a start time
    std::vector<data::ChangeAction> task_changes;
    std::vector<data::ChangeAction> process_changes;
    process_changes.reserve(occ_to_add.size());

    ProcessHandler::ProcessConstIterator process_ref_itr;
    assert(process_handler_->find(first_occ.id, process_ref_itr));

    // reserve task changes size
    task_changes.reserve((occ_to_add.size()) * process_ref_itr->second->graph.size());

    // Generate and add process and tasks occurrences
    for (const auto & occ_itr : occ_to_add) {
      // Create copy of process
      auto process_to_add = data::Process::make_shared(*process_ref_itr->second);

      // Update process data
      process_to_add->series_id = series_to_update->id();
      process_to_add->id = occ_itr.id;

      // create and add tasks
      auto nodes = process_ref_itr->second->graph.get_all_nodes();
      for (const auto & n : nodes) {
        // Find original task
        TaskHandler::TaskConstIterator task_itr;
        assert(task_handler_->find(n.first, task_itr));

        // generate and replace graph node with new id
        const auto new_id = data::gen_uuid();
        process_to_add->graph.update_node(task_itr->first, new_id);

        // Create and add task
        auto task_to_add = data::Task::make_shared(*task_itr->second);
        task_to_add->id = new_id;
        task_to_add->process_id = process_to_add->id;
        task_to_add->start_time = occ_itr.time;
        task_handler_->emplace(task_to_add);
        task_changes.push_back({task_to_add->id, "add"});
      }
      process_handler_->emplace(process_to_add);
      process_changes.push_back({process_to_add->id, "add"});
    }
    record_.add("task", task_changes);
    record_.add("process", process_changes);
  } else {
    assert(false);
  }
  series_handler_->replace(itr, series_to_update);
  record_.add("series", {{series_to_update->id(), "update"}});
}

// CURRENTLY NOT SUPPORTED
bool SeriesAction::_validate_series_update_cron(
  const SeriesHandler::ConstPtr & series_handler,
  std::string & error)
{
  // TODO(Anyone) : Implement this feature
  error = "SERIES_UPDATE_CRON is currently not supported.";
  return false;
}

void SeriesAction::_apply_series_update_cron()
{
  // TODO(Anyone) : Implement this feature
  // Currently not suppported
  throw std::runtime_error("Invalid action type");
}

// CURRENTLY NOT SUPPORTED
bool SeriesAction::_validate_series_update_until(
  const SeriesHandler::ConstPtr & series_handler,
  std::string & error)
{
  error = "SERIES_UPDATE_UNTIL currently not supported.";
  return false;
}

void SeriesAction::_apply_series_update_until()
{
  // TODO(Anyone) : Implement this feature
  // Currently not suppported
  throw std::runtime_error("Invalid action type");
}

// CURRENTLY NOT SUPPORTED
bool SeriesAction::_validate_series_update_occurrence(
  const SeriesHandler::ConstPtr & series_handler,
  std::string & error)
{
  // TODO(Anyone) : Implement this feature
  error = "SERIES_UPDATE_OCCURRENCE currently not supported.";
  return false;
}

void SeriesAction::_apply_series_update_occurrence()
{
  // TODO(Anyone) : Implement this feature
  // Currently not suppported
  throw std::runtime_error("Invalid action type");
}

bool SeriesAction::_validate_series_update_occurrence_time(
  const SeriesHandler::ConstPtr & series_handler,
  const TaskHandler::ConstPtr & task_handler,
  const ProcessHandler::ConstPtr & process_handler,
  std::string & error)
{
  if (!data_.id.has_value() || !data_.occurrence_id.has_value() ||
    !data_.occurrence_time.has_value())
  {
    error =
      "SERIES_UPDATE_OCCURRENCE_TIME failed, id, occurrence_id and time must be set in payload.";
    return false;
  }

  SeriesHandler::SeriesConstIterator itr;
  if (!series_handler->find(data_.id.value(), itr)) {
    error = "SERIES_UPDATE_OCCURRENCE_TIME failed, series [" + data_.id.value() +
      "] does not exist.";
    return false;
  }

  // Check if occurrence exists in series
  data::Occurrence occ;
  try {
    occ = itr->second->get_occurrence(data_.occurrence_id.value());
  } catch (const std::out_of_range & e) {
    error = "SERIES_UPDATE_OCCURRENCE_TIME failed, occurrence [" + data_.occurrence_id.value() +
      "] does not exist.";
    return false;
  }

  // Confirm if occurrence task/process exists
  if (itr->second->type() == "process") {
    ProcessHandler::ProcessConstIterator process_itr;
    if (!process_handler->find(occ.id, process_itr)) {
      error = "SERIES_UPDATE_OCCURRENCE_TIME failed, process [" + data_.occurrence_id.value() +
        "] does not exist.";
      return false;
    }
  } else if (itr->second->type() == "task") {
    TaskHandler::TaskConstIterator task_itr;
    if (!task_handler->find(occ.id, task_itr)) {
      error = "SERIES_UPDATE_OCCURRENCE_TIME failed, task [" + data_.occurrence_id.value() +
        "] does not exist.";
      return false;
    }
  } else {
    assert(false);
  }

  // Check if adjusted time is valid
  auto prev_occ = itr->second->get_prev_occurrence(occ.time);
  // Check if time is before or equirvalent to the previous occ time.
  if (prev_occ.has_value()) {
    if (data_.occurrence_time.value() <= prev_occ.value().time) {
      error = "SERIES_UPDATE_OCCURRENCE_TIME failed, new update time [" +
        std::string(data_.occurrence_time.value().to_localtime()) +
        "] is before or equivalent to previous occurrence time at [" +
        std::string(prev_occ.value().time.to_localtime()) + "].";
      return false;
    }
  }
  auto next_occ = itr->second->get_next_occurrence(occ.time);
  if (next_occ.has_value()) {
    if (data_.occurrence_time.value() >= next_occ.value().time) {
      error = "SERIES_UPDATE_OCCURRENCE_TIME failed, new update time [" +
        std::string(data_.occurrence_time.value().to_localtime()) +
        "] is after or equivalent to next occurrence time at [" +
        std::string(next_occ.value().time.to_localtime()) + "].";
      return false;
    }
  }
  return true;
}

void SeriesAction::_apply_series_update_occurrence_time()
{
  // update series occurrence
  SeriesHandler::SeriesConstIterator itr;
  assert(series_handler_->find(data_.id.value(), itr));
  const auto occ = itr->second->get_occurrence(data_.occurrence_id.value());
  auto series_to_update = data::Series::make_shared(*itr->second);
  series_to_update->update_occurrence(occ.time, data_.occurrence_time.value());
  series_handler_->emplace(series_to_update);
  record_.add("series", {{series_to_update->id(), "update"}});
  // update task or process
  if (itr->second->type() == "task") {
    TaskHandler::TaskConstIterator task_itr;
    assert(task_handler_->find(data_.occurrence_id.value(), task_itr));
    auto task_update = data::Task::make_shared(*task_itr->second);
    task_update->start_time = data_.occurrence_time.value();
    task_handler_->replace(task_itr, task_update);
    record_.add("task", {{task_itr->first, "update"}});
  } else if (itr->second->type() == "process") {
    ProcessHandler::ProcessConstIterator process_itr;
    assert(process_handler_->find(data_.occurrence_id.value(), process_itr));
    const auto nodes = process_itr->second->graph.get_all_nodes();
    std::vector<data::ChangeAction> task_changes;
    task_changes.reserve(nodes.size());
    for (const auto & n : nodes) {
      TaskHandler::TaskConstIterator task_itr;
      assert(task_handler_->find(n.first, task_itr));
      auto task_update = data::Task::make_shared(*task_itr->second);
      task_update->start_time = data_.occurrence_time.value();
      task_handler_->emplace(task_update);
      task_changes.push_back({task_itr->first, "update"});
    }
    record_.add("task", task_changes);
  } else {
    assert(false);
  }
  return;
}

bool SeriesAction::_validate_series_delete_occurrence(
  const SeriesHandler::ConstPtr & series_handler,
  const TaskHandler::ConstPtr & task_handler,
  const ProcessHandler::ConstPtr & process_handler,
  std::string & error)
{
  if (!data_.id.has_value() || !data_.occurrence_id.has_value()) {
    error = "SERIES_DELETE_OCCURRENCE failed, id and occurrence_id must be set in payload.";
    return false;
  }
  SeriesHandler::SeriesConstIterator itr;
  if (!series_handler->find(data_.id.value(), itr)) {
    error = "SERIES_DELETE_OCCURRENCE failed, series [" + data_.id.value() +
      "] does not exist.";
    return false;
  }
  // Check if occurrence exists in series
  data::Occurrence occ;
  try {
    occ = itr->second->get_occurrence(data_.occurrence_id.value());
  } catch (const std::out_of_range & e) {
    error = "SERIES_DELETE_OCCURRENCE failed, occurrence [" + data_.occurrence_id.value() +
      "] does not exist.";
    return false;
  }

  // Confirm if occurrence task/process exists
  if (itr->second->type() == "process") {
    ProcessHandler::ProcessConstIterator process_itr;
    if (!process_handler->find(occ.id, process_itr)) {
      error = "SERIES_DELETE_OCCURRENCE failed, process [" + data_.occurrence_id.value() +
        "] does not exist in the Process Handler.";
      return false;
    }
  } else if (itr->second->type() == "task") {
    TaskHandler::TaskConstIterator task_itr;
    if (!task_handler->find(occ.id, task_itr)) {
      error = "SERIES_DELETE_OCCURRENCE failed, task [" + data_.occurrence_id.value() +
        "] does not exist in the Task Handler.";
      return false;
    }
  } else {
    // We should not reach here as the series should not be created with
    // another type name.
    assert(false);
  }
  return true;
}

void SeriesAction::_apply_series_delete_occurrence()
{
  SeriesHandler::SeriesConstIterator series_itr;
  assert(series_handler_->find(data_.id.value(), series_itr));
  auto occ_to_delete =
    series_itr->second->get_occurrence(data_.occurrence_id.value());

  // Delete task or process
  if (series_itr->second->type() == "task") {
    std::vector<data::ChangeAction> task_changes;
    task_changes.reserve(1);
    TaskHandler::TaskConstIterator task_itr;
    assert(task_handler_->find(occ_to_delete.id, task_itr));
    task_handler_->erase(task_itr);
    task_changes.push_back({occ_to_delete.id, "delete"});
    record_.add("task", task_changes);
  } else if (series_itr->second->type() == "process") {
    std::vector<data::ChangeAction> process_changes;
    process_changes.reserve(1);
    ProcessHandler::ProcessConstIterator process_itr;
    assert(process_handler_->find(occ_to_delete.id, process_itr));
    process_handler_->erase(process_itr);
    process_changes.push_back({occ_to_delete.id, "delete"});
    record_.add("process", process_changes);

    // delete tasks in process
    auto nodes = process_itr->second->graph.get_all_nodes();
    std::vector<data::ChangeAction> task_changes;
    task_changes.reserve(nodes.size());
    for (const auto & n : nodes) {
      TaskHandler::TaskConstIterator task_itr;
      assert(task_handler_->find(n.first, task_itr));
      task_handler_->erase(task_itr);
      task_changes.push_back({n.first, "delete"});
    }
    record_.add("task", task_changes);
  } else {
    assert(false);
  }
  // Delete occurrence in the series
  auto series_to_update = data::Series::make_shared(*series_itr->second);
  series_to_update->delete_occurrence(occ_to_delete.time);
  series_handler_->emplace(series_to_update);
  data::ChangeAction series_change = {series_itr->first, "update"};
  record_.add("series", {series_change});
  return;
}


bool SeriesAction::_validate_series_delete(
  const SeriesHandler::ConstPtr & series_handler,
  std::string & error)
{
  if (!data_.id.has_value()) {
    error = "SERIES_DELETE failed, id must be set in payload.";
    return false;
  }

  SeriesHandler::SeriesConstIterator itr;
  if (!series_handler->find(data_.id.value(), itr)) {
    error = "SERIES_DELETE failed, series [" + data_.id.value() +
      "] does not exist.";
    return false;
  }

  return true;
}

void SeriesAction::_apply_series_delete_series()
{
  SeriesHandler::SeriesConstIterator series_itr;
  assert(series_handler_->find(data_.id.value(), series_itr));
  const auto occurrences = series_itr->second->occurrences();

  // Delete all occurrences
  if (series_itr->second->type() == "task") {
    std::vector<data::ChangeAction> task_changes;
    task_changes.reserve(occurrences.size());
    for (const auto & occ : occurrences) {
      TaskHandler::TaskConstIterator task_itr;
      assert(task_handler_->find(occ.id, task_itr));
      task_handler_->erase(task_itr);
      task_changes.push_back({occ.id, "delete"});
    }
    record_.add("task", task_changes);
  } else if (series_itr->second->type() == "process") {
    std::vector<data::ChangeAction> process_changes;
    process_changes.reserve(occurrences.size());
    std::vector<ProcessHandler::ProcessConstIterator> processes_to_erase;

    for (const auto & occ : occurrences) {
      ProcessHandler::ProcessConstIterator process_itr;
      assert(process_handler_->find(occ.id, process_itr));
      // update tasks in process
      auto nodes = process_itr->second->graph.get_all_nodes();
      std::vector<data::ChangeAction> task_changes;
      task_changes.reserve(nodes.size());
      for (const auto & n : nodes) {
        TaskHandler::TaskConstIterator task_itr;
        assert(task_handler_->find(n.first, task_itr));
        task_changes.push_back({n.first, "delete"});
        task_handler_->erase(task_itr);
      }
      record_.add("task", task_changes);
      process_changes.push_back({occ.id, "delete"});
      process_handler_->erase(process_itr);
      processes_to_erase.push_back(process_itr);
    }
    record_.add("process", process_changes);
  } else {
    assert(false);
  }

  // delete series
  data::ChangeAction series_change = {series_itr->first, "delete"};
  record_.add("series", {series_change});
  series_handler_->erase(series_itr);
  return;
}

bool SeriesAction::_validate_series_delete_all(
  const SeriesHandler::ConstPtr & series_handler,
  std::string & error)
{
  // This just deletes all series.
  return true;
}

void SeriesAction::_apply_series_delete_all()
{
  return;
}

}  // namespace cache
}  // namespace rmf2_scheduler
