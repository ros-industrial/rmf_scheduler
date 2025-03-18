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

#include "rmf2_scheduler/cache/process_action.hpp"
#include "rmf2_scheduler/utils/dag_helper.hpp"

namespace rmf2_scheduler
{

namespace cache
{

ProcessAction::ProcessAction(
  const std::string & type,
  const ActionPayload & payload
)
: Action(type, payload)
{
}

ProcessAction::~ProcessAction()
{
}

bool ProcessAction::validate(
  const ScheduleCache::Ptr & cache,
  std::string & error
)
{
  if (!on_validate_prepare(error)) {
    return false;
  }

  // Check if action type is valid
  auto process_action_type = data_.type;
  if (process_action_type.find(data::action_type::PROCESS_PREFIX) != 0) {
    std::ostringstream oss;
    oss << "PROCESS action invalid [" << process_action_type << "].";
    error = oss.str();
    return false;
  }

  bool result = false;

  // Create task handler
  ScheduleCache::TaskRestricted task_token;
  auto task_handler = cache->make_task_handler(task_token);

  // Create task handler
  ScheduleCache::ProcessRestricted process_token;
  auto process_handler = cache->make_process_handler(process_token);

  if (process_action_type == data::action_type::PROCESS_ADD) {
    result = _validate_process_add(process_handler, task_handler, error);
  } else if (process_action_type == data::action_type::PROCESS_ATTACH_NODE) {
    result = _validate_process_attach_node(process_handler, task_handler, error);
  } else if (process_action_type == data::action_type::PROCESS_ADD_DEPENDENCY) {
    result = _validate_process_add_dependency(process_handler, task_handler, error);
  } else if (process_action_type == data::action_type::PROCESS_UPDATE) {
    result = _validate_process_update(process_handler, task_handler, error);
  } else if (process_action_type == data::action_type::PROCESS_UPDATE_START_TIME) {
    result = _validate_process_update_start_time(process_handler, error);
  } else if (process_action_type == data::action_type::PROCESS_DETACH_NODE) {
    result = _validate_process_detach_node(process_handler, task_handler, error);
  } else if (process_action_type == data::action_type::PROCESS_DELETE_DEPENDENCY) {
    result = _validate_process_delete_dependency(process_handler, task_handler, error);
  } else if (process_action_type == data::action_type::PROCESS_DELETE) {
    result = _validate_process_delete(process_handler, task_handler, error);
  } else if (process_action_type == data::action_type::PROCESS_DELETE_ALL) {
    result = _validate_process_delete(process_handler, task_handler, error);
  } else {
    std::ostringstream oss;
    oss << "Undefined PROCESS action [" << process_action_type << "].";
    error = oss.str();
    return false;
  }

  if (result) {
    process_handler_ = process_handler;
    task_handler_ = task_handler;
    on_validate_complete();
  }

  return result;
}

void ProcessAction::apply()
{
  on_apply_prepare();

  auto process_action_type = data_.type;

  if (process_action_type == data::action_type::PROCESS_ADD) {
    _apply_process_add();
  } else if (process_action_type == data::action_type::PROCESS_ATTACH_NODE) {
    _apply_process_attach_node();
  } else if (process_action_type == data::action_type::PROCESS_ADD_DEPENDENCY) {
    _apply_process_add_dependency();
  } else if (process_action_type == data::action_type::PROCESS_UPDATE) {
    _apply_process_update();
  } else if (process_action_type == data::action_type::PROCESS_UPDATE_START_TIME) {
    _apply_process_update_start_time();
  } else if (process_action_type == data::action_type::PROCESS_DETACH_NODE) {
    _apply_process_detach_node();
  } else if (process_action_type == data::action_type::PROCESS_DELETE_DEPENDENCY) {
    _apply_process_delete_dependency();
  } else if (process_action_type == data::action_type::PROCESS_DELETE) {
    _apply_process_delete();
  } else if (process_action_type == data::action_type::PROCESS_DELETE_ALL) {
    _apply_process_delete_all();
  } else {
    assert(false);
  }

  on_apply_complete();
}

bool ProcessAction::_validate_process_add(
  const ProcessHandler::ConstPtr & process_handler,
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  auto process = data_.process;
  if (!process) {
    error = "PROCESS_ADD failed, no process found in payload.";
    return false;
  }

  ProcessHandler::ProcessConstIterator itr;
  if (process_handler->find(process->id, itr)) {
    error = "PROCESS_ADD failed, process [" + process->id + "] already exists.";
    return false;
  }

  // Validate all IDs in the graph exists
  std::vector<TaskHandler::TaskConstIterator> task_itrs;
  auto all_nodes = process->graph.get_all_nodes_ordered();
  task_itrs.reserve(all_nodes.size());
  for (auto & node_itr : all_nodes) {
    TaskHandler::TaskConstIterator task_itr;
    bool result = task_handler->find(node_itr.first, task_itr);
    if (!result) {
      error = "PROCESS_ADD failed, node [" + node_itr.first + "] doesn't exist as task.";
      return false;
    }

    // Check if the event is already part of a process
    if (task_itr->second->process_id.has_value()) {
      error = "PROCESS_ADD failed, task [" + node_itr.first +
        "] is already part of a process, please detach it first.";
      return false;
    }
    task_itrs.push_back(task_itr);
  }

  task_itrs_ = task_itrs;
  return true;
}

void ProcessAction::_apply_process_add()
{
  auto process = data_.process;

  // Overwrite the process IDs
  std::vector<data::ChangeAction> task_changes;
  task_changes.reserve(task_itrs_.size());
  for (auto & task_itr : task_itrs_) {
    task_itr->second->process_id = process->id;
    task_changes.push_back({task_itr->first, "update"});
  }
  record_.add("task", task_changes);

  // Add the process
  process_handler_->emplace(process);
  record_.add("process", {{process->id, "add"}});
}

bool ProcessAction::_validate_process_attach_node(
  const ProcessHandler::ConstPtr & process_handler,
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  if (!data_.id.has_value()) {
    error = "PROCESS_ATTACH_NODE failed, no process ID found in payload.";
    return false;
  }

  auto process_id = data_.id.value();

  if (!data_.node_id.has_value()) {
    error = "PROCESS_ATTACH_NODE failed, no node found in payload.";
    return false;
  }

  auto node_id = data_.node_id.value();

  // Check if the process exist
  ProcessHandler::ProcessConstIterator process_itr;
  if (!process_handler->find(process_id, process_itr)) {
    error = "PROCESS_ATTACH_NODE failed, process [" + process_id + "] doesn't exist.";
    return false;
  }

  if (process_itr->second->graph.has_node(node_id)) {
    error = "PROCESS_ATTACH_NODE failed, node [" + node_id + "] is already part of the process.";
    return false;
  }

  TaskHandler::TaskConstIterator task_itr;
  if (!task_handler->find(node_id, task_itr)) {
    error = "PROCESS_ATTACH_NODE failed, node [" + node_id + "] doesn't exist as task.";
    return false;
  }

  // Check if the task is already part of a process
  if (task_itr->second->process_id.has_value()) {
    error = "PROCESS_ATTACH_NODE failed, task [" + task_itr->first +
      "] is already part of another process, please detach it first.";
    return false;
  }

  task_itrs_ = {task_itr};
  process_itr_ = process_itr;
  return true;
}

void ProcessAction::_apply_process_attach_node()
{
  assert(task_itrs_.size() == 1);

  // Update the task
  task_itrs_[0]->second->process_id = process_itr_->second->id;
  record_.add("task", {{task_itrs_[0]->first, "update"}});

  // Attach the node to the process
  data::Process::Ptr process_to_update = std::make_shared<data::Process>(
    *process_itr_->second
  );
  process_to_update->graph.add_node(task_itrs_[0]->second->id);
  process_handler_->replace(process_itr_, process_to_update);
  record_.add("process", {{process_to_update->id, "update"}});
}

bool ProcessAction::_validate_process_add_dependency(
  const ProcessHandler::ConstPtr & process_handler,
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  // Check if all payloads exist
  if (!data_.source_id.has_value()) {
    error = "PROCESS_ADD_DEPENDENCY failed, no source found in payload.";
    return false;
  }
  auto source_id = data_.source_id.value();

  if (!data_.destination_id.has_value()) {
    error = "PROCESS_ADD_DEPENDENCY failed, no destination found in payload.";
    return false;
  }
  auto destination_id = data_.destination_id.value();

  if (!data_.edge_type.has_value()) {
    error = "PROCESS_ADD_DEPENDENCY failed, no edge property found in payload.";
    return false;
  }
  auto edge_type = data_.edge_type.value();

  // Check if the IDs are tasks in the same process
  TaskHandler::TaskConstIterator source_task_itr;
  if (!task_handler->find(source_id, source_task_itr)) {
    error = "PROCESS_ADD_DEPENDENCY failed, node [" + source_id + "] doesn't exist as task.";
    return false;
  }

  TaskHandler::TaskConstIterator destination_task_itr;
  if (!task_handler->find(destination_id, destination_task_itr)) {
    error = "PROCESS_ADD_DEPENDENCY failed, node [" +
      destination_id + "] doesn't exist as task.";
    return false;
  }

  if (!source_task_itr->second->process_id.has_value() ||
    !destination_task_itr->second->process_id.has_value() ||
    source_task_itr->second->process_id != destination_task_itr->second->process_id)
  {
    error = "PROCESS_ADD_DEPENDENCY failed, source and destination node "
      "doesn't belong to the same process.";
    return false;
  }

  ProcessHandler::ProcessConstIterator process_itr;
  bool result = process_handler->find(
    *source_task_itr->second->process_id,
    process_itr
  );
  assert(result);

  // Check if edge already exists
  auto successors = process_itr->second->graph.get_node(source_id)->outbound_edges();
  if (successors.find(destination_id) != successors.end()) {
    error = "PROCESS_ADD_DEPENDENCY failed, edge [" + source_id + "] to [" +
      destination_id + "] already exists.";
    return false;
  }

  task_itrs_ = {source_task_itr, destination_task_itr};
  process_itr_ = process_itr;

  assert(process_itr_->second->graph.has_node(source_id));
  assert(process_itr_->second->graph.has_node(destination_id));
  return true;
}

void ProcessAction::_apply_process_add_dependency()
{
  assert(task_itrs_.size() == 2);

  // Add the dependency to the process
  data::Process::Ptr process_to_update = std::make_shared<data::Process>(
    *process_itr_->second
  );
  process_to_update->graph.add_edge(
    task_itrs_[0]->second->id,
    task_itrs_[1]->second->id,
    data::Edge(data_.edge_type.value())
  );
  process_handler_->replace(process_itr_, process_to_update);
  record_.add("process", {{process_to_update->id, "update"}});
}

bool ProcessAction::_validate_process_update(
  const ProcessHandler::ConstPtr & process_handler,
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  auto process = data_.process;
  if (!process) {
    error = "PROCESS_UPDATE failed, no process found in payload.";
    return false;
  }

  ProcessHandler::ProcessConstIterator process_itr;
  if (!process_handler->find(process->id, process_itr)) {
    error = "PROCESS_UPDATE failed, process [" + process->id + "] doesn't exist.";
    return false;
  }

  // Validate all IDs in the graph exists
  std::vector<TaskHandler::TaskConstIterator> task_itrs;
  auto all_nodes = process->graph.get_all_nodes_ordered();
  task_itrs.reserve(all_nodes.size());
  for (auto & node_itr : all_nodes) {
    TaskHandler::TaskConstIterator task_itr;
    bool result = task_handler->find(node_itr.first, task_itr);
    if (!result) {
      error = "PROCESS_UPDATE failed, node [" + node_itr.first + "] doesn't exist as task.";
      return false;
    }

    // Task to add to the process
    if (!task_itr->second->process_id.has_value()) {
      task_itrs.push_back(task_itr);
      continue;
    }

    // Check if the task is in the same process
    if (*task_itr->second->process_id != process->id) {
      error = "PROCESS_UPDATE failed, task [" + node_itr.first +
        "] is part of another process, please detach it first.";
      return false;
    }
  }

  // Check if there are tasks to be detached
  std::vector<TaskHandler::TaskConstIterator> extra_task_itrs;
  auto old_nodes = process_itr->second->graph.get_all_nodes_ordered();
  assert(old_nodes.size() + task_itrs.size() >= all_nodes.size());
  if (old_nodes.size() + task_itrs.size() > all_nodes.size()) {
    extra_task_itrs.reserve(old_nodes.size() + task_itrs.size() - all_nodes.size());
    for (auto & node_itr : old_nodes) {
      // Append the task to detach to the extra tasks
      if (all_nodes.find(node_itr.first) == all_nodes.end()) {
        TaskHandler::TaskConstIterator task_itr;
        bool result = task_handler->find(node_itr.first, task_itr);
        assert(result);
        extra_task_itrs.push_back(task_itr);
      }
    }
  }

  process_itr_ = process_itr;
  task_itrs_ = task_itrs;
  extra_task_itrs_ = extra_task_itrs;
  return true;
}

void ProcessAction::_apply_process_update()
{
  auto process = data_.process;

  // Overwrite the process IDs
  std::vector<data::ChangeAction> task_changes;
  task_changes.reserve(task_itrs_.size() + extra_task_itrs_.size());
  for (auto & task_itr : task_itrs_) {
    task_itr->second->process_id = process->id;
    task_changes.push_back({task_itr->first, "update"});
  }

  // Remove the process IDs
  for (auto & task_itr : extra_task_itrs_) {
    task_itr->second->process_id.reset();
    task_changes.push_back({task_itr->first, "update"});
  }
  record_.add("task", task_changes);

  // Add the process
  process_handler_->replace(process_itr_, process);
  record_.add("process", {{process->id, "update"}});
}

bool ProcessAction::_validate_process_update_start_time(
  const ProcessHandler::ConstPtr & process_handler,
  std::string & error
)
{
  // Check if payloads exist.
  if (!data_.id.has_value()) {
    error = "PROCESS_UPDATE_START_TIME failed, no process ID found in payload.";
    return false;
  }
  auto process_id = data_.id.value();

  // Check if the process exist
  ProcessHandler::ProcessConstIterator process_itr;
  if (!process_handler->find(process_id, process_itr)) {
    error = "PROCESS_UPDATE_START_TIME failed, process [" + process_id + "] doesn't exist.";
    return false;
  }

  // Check if the process is cyclic
  if (!utils::is_valid_dag(process_itr->second->graph)) {
    error = "PROCESS_UPDATE_START_TIME failed, process [" + process_id + "] is cyclic.";
    return false;
  }

  process_itr_ = process_itr;
  return true;
}

void ProcessAction::_apply_process_update_start_time()
{
  process_handler_->update_start_time(process_itr_);

  // Record the events changed
  std::vector<data::ChangeAction> task_changes;
  auto all_nodes = process_itr_->second->graph.get_all_nodes_ordered();

  task_changes.reserve(all_nodes.size());
  for (auto & node_itr : all_nodes) {
    task_changes.push_back({node_itr.first, "update"});
  }
  record_.add("task", task_changes);
}

bool ProcessAction::_validate_process_detach_node(
  const ProcessHandler::ConstPtr & process_handler,
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  if (!data_.node_id.has_value()) {
    error = "PROCESS_DETACH_NODE failed, no node found in payload.";
    return false;
  }
  auto node_id = data_.node_id.value();

  // Check if the node exist as task
  TaskHandler::TaskConstIterator task_itr;
  if (!task_handler->find(node_id, task_itr)) {
    error = "PROCESS_DETACH_NODE failed, node [" + node_id + "] doesn't exist as task.";
    return false;
  }

  // Check if the node belongs to a process
  if (!task_itr->second->process_id.has_value()) {
    error = "PROCESS_DETACH_NODE failed, node [" + node_id + "] doesn't belong to a process.";
    return false;
  }

  task_itrs_ = {task_itr};

  bool result;
  result = process_handler->find(*task_itr->second->process_id, process_itr_);
  assert(result);
  return true;
}

void ProcessAction::_apply_process_detach_node()
{
  assert(task_itrs_.size() == 1);

  // Update the task
  task_itrs_[0]->second->process_id.reset();
  record_.add("task", {{task_itrs_[0]->first, "update"}});

  // Detach the node to the process
  data::Process::Ptr process_to_update = std::make_shared<data::Process>(
    *process_itr_->second
  );
  process_to_update->graph.delete_node(task_itrs_[0]->second->id);
  process_handler_->replace(process_itr_, process_to_update);
  record_.add("process", {{process_to_update->id, "update"}});
}

bool ProcessAction::_validate_process_delete_dependency(
  const ProcessHandler::ConstPtr & process_handler,
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  // Check if all payloads exist
  if (!data_.source_id.has_value()) {
    error = "PROCESS_DELETE_DEPENDENCY failed, no source found in payload.";
    return false;
  }
  auto source_id = data_.source_id.value();

  if (!data_.destination_id.has_value()) {
    error = "PROCESS_DELETE_DEPENDENCY failed, no destination found in payload.";
    return false;
  }
  auto destination_id = data_.destination_id.value();

  // Check if the IDs are tasks in the same process
  TaskHandler::TaskConstIterator source_task_itr;
  if (!task_handler->find(source_id, source_task_itr)) {
    error = "PROCESS_DELETE_DEPENDENCY failed, node [" + source_id + "] doesn't exist as task.";
    return false;
  }

  TaskHandler::TaskConstIterator destination_task_itr;
  if (!task_handler->find(destination_id, destination_task_itr)) {
    error = "PROCESS_DELETE_DEPENDENCY failed, node [" +
      destination_id + "] doesn't exist as task.";
    return false;
  }

  if (!source_task_itr->second->process_id.has_value() ||
    !destination_task_itr->second->process_id.has_value() ||
    source_task_itr->second->process_id != destination_task_itr->second->process_id)
  {
    error = "PROCESS_DELETE_DEPENDENCY failed, source and destination node "
      "doesn't belong to the same process.";
    return false;
  }

  ProcessHandler::ProcessConstIterator process_itr;
  bool result = process_handler->find(
    *source_task_itr->second->process_id,
    process_itr
  );
  assert(result);

  // Check if edge already exists
  auto successors = process_itr->second->graph.get_node(source_id)->outbound_edges();
  if (successors.find(destination_id) == successors.end()) {
    error = "PROCESS_DELETE_DEPENDENCY failed, edge [" + source_id + "] to [" +
      destination_id + "] doesn't exist.";
    return false;
  }

  task_itrs_ = {source_task_itr, destination_task_itr};
  process_itr_ = process_itr;

  assert(process_itr_->second->graph.has_node(source_id));
  assert(process_itr_->second->graph.has_node(destination_id));
  return true;
}

void ProcessAction::_apply_process_delete_dependency()
{
  assert(task_itrs_.size() == 2);

  // Remove the dependency from the process
  data::Process::Ptr process_to_update = std::make_shared<data::Process>(
    *process_itr_->second
  );
  process_to_update->graph.delete_edge(
    task_itrs_[0]->second->id,
    task_itrs_[1]->second->id
  );
  process_handler_->replace(process_itr_, process_to_update);

  record_.add("process", {{process_to_update->id, "update"}});
}

bool ProcessAction::_validate_process_delete(
  const ProcessHandler::ConstPtr & process_handler,
  const TaskHandler::ConstPtr & task_handler,
  std::string & error
)
{
  if (!data_.id.has_value()) {
    error = "PROCESS_DELETE failed, no ID found in payload.";
    return false;
  }
  auto id = data_.id.value();

  ProcessHandler::ProcessConstIterator process_itr;
  bool process_exist = process_handler->find(id, process_itr);

  if (!process_exist) {
    error = "PROCESS_DELETE failed, process [" + id + "] doesn't exist.";
    return false;
  }

  std::vector<TaskHandler::TaskConstIterator> task_itrs;
  auto all_nodes = process_itr->second->graph.get_all_nodes_ordered();
  task_itrs.reserve(all_nodes.size());
  for (auto & node_itr : all_nodes) {
    TaskHandler::TaskConstIterator task_itr;
    bool result = task_handler->find(node_itr.first, task_itr);
    assert(result);
    assert(task_itr->second->process_id.has_value());
    assert(*task_itr->second->process_id == id);

    task_itrs.push_back(task_itr);
  }

  task_itrs_ = task_itrs;
  process_itr_ = process_itr;
  return true;
}

void ProcessAction::_apply_process_delete()
{
  // Overwrite the process IDs
  std::vector<data::ChangeAction> task_changes;
  task_changes.reserve(task_itrs_.size());
  for (auto & task_itr : task_itrs_) {
    task_itr->second->process_id.reset();
    task_changes.push_back({task_itr->first, "update"});
  }
  record_.add("task", task_changes);

  // delete the process
  record_.add("process", {{process_itr_->second->id, "delete"}});
  process_handler_->erase(process_itr_);
}

void ProcessAction::_apply_process_delete_all()
{
  // delete the tasks
  std::vector<data::ChangeAction> task_changes;
  task_changes.reserve(task_itrs_.size());
  for (auto & task_itr : task_itrs_) {
    task_changes.push_back({task_itr->second->id, "delete"});
    task_handler_->erase(task_itr);
  }
  record_.add("task", task_changes);

  // delete the process
  record_.add("process", {{process_itr_->second->id, "delete"}});
  process_handler_->erase(process_itr_);
}

}  // namespace cache
}  // namespace rmf2_scheduler
