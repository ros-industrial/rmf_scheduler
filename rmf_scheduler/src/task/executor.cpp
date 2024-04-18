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

#include "rmf_scheduler/task/executor.hpp"
#include "rmf_scheduler/task/task_plugin_impl.hpp"

namespace rmf_scheduler
{

namespace task
{

class ExecutionStatusCompletionObserver : public ExecutionObserverBase
{
public:
  ExecutionStatusCompletionObserver(
    std::mutex & status_mtx,
    std::unordered_map<std::string, Executor::Status> & status)
  : mtx_(status_mtx), status_(status)
  {
  }

  std::shared_future<Executor::Status> get_future(const std::string & id)
  {
    ongoing_task_sig_[id] = std::promise<Executor::Status>();
    return ongoing_task_sig_.at(id).get_future();
  }

  void completion_callback(
    const std::string & id,
    bool success,
    const std::string & detail = "") final
  {
    std::lock_guard<std::mutex> lk(mtx_);
    RS_LOG_INFO("Calling default completion callback");
    auto & status = status_[id];
    status = Executor::Status {
      success ? Executor::State::COMPLETED : Executor::State::FAILED,
      "",
      detail
    };
    auto itr = ongoing_task_sig_.find(id);
    if (itr != ongoing_task_sig_.end()) {
      itr->second.set_value(status);
      ongoing_task_sig_.erase(itr);
    }
  }

  void cancel_callback(
    const std::string & id)
  {
    const auto & status = status_[id];
    auto itr = ongoing_task_sig_.find(id);
    itr->second.set_value(status);
    ongoing_task_sig_.erase(itr);
  }

  void update(
    const std::string & /*id*/,
    uint64_t /*remaining_time*/) final
  {
  }

private:
  std::mutex & mtx_;
  std::unordered_map<std::string, Executor::Status> & status_;
  std::unordered_map<std::string, std::promise<Executor::Status>> ongoing_task_sig_;
};

Executor::Executor()
{
  runtime_status_completion_observer_ = std::make_shared<ExecutionStatusCompletionObserver>(
    mtx_, status_);
}

void Executor::load_plugin(
  const std::shared_ptr<void> & node,
  const std::string & name,
  const std::string & plugin,
  const std::vector<std::string> & supported_tasks)
{
  auto new_plugin = load_task_plugin_impl<ExecutionInterface>(
    this, node, name, plugin, "rmf_scheduler::task::ExecutionInterface", supported_tasks);
  // Attach the completion observer first always, so it will be executed last
  new_plugin->attach(runtime_status_completion_observer_);

  // Attach other observer
  for (auto observer : observers_) {
    new_plugin->attach(observer);
  }
}

void Executor::unload_plugin(
  const std::string & name)
{
  TaskPluginManager::unload_plugin(name);
}

std::shared_future<Executor::Status> Executor::run_async(
  const data::Event & event)
{
  if (!is_supported(event.type)) {
    throw exception::TaskExecutionException(
            "No runtime interface configured for task type [%s]",
            event.type.c_str());
  }

  auto plugin = get_supported_plugin<ExecutionInterface>(event.type);

  std::lock_guard lk(mtx_);
  nlohmann::json task_details_json;
  try {
    task_details_json = nlohmann::json::parse(event.task_details);
  } catch (const std::exception & e) {
    throw exception::TaskExecutionException(
            "task details for event [%s] is not valid JSON: %s. "
            "Most likely the task builder output is invalid.\n%s",
            event.id.c_str(),
            event.task_details.c_str(),
            e.what());
  }

  try {
    plugin.second->start(
      event.id,
      task_details_json);
  } catch (const std::exception & e) {
    throw exception::TaskExecutionException(
            "Execution plugin [%s] cannot start executing event [%s].\n"
            "Task detail:\n%s.\n"
            "Plugin Error message:\n%s",
            plugin.first.c_str(),
            event.id.c_str(),
            event.task_details.c_str(),
            e.what());
  }
  status_[event.id] = Status{State::ONGOING, plugin.first, ""};

  auto completion_observer = std::dynamic_pointer_cast<ExecutionStatusCompletionObserver>(
    runtime_status_completion_observer_);

  return completion_observer->get_future(event.id);
}

void Executor::pause(const std::string & id)
{
  std::lock_guard lk(mtx_);
  auto itr = status_.find(id);
  if (itr == status_.end()) {
    throw exception::TaskExecutionException("Task %s not started, cannot pause.", id.c_str());
  }

  if (itr->second.state != State::ONGOING) {
    // ignore if the task is not ongoing
    return;
  }

  get_plugin<ExecutionInterface>(itr->second.runtime_interface)->pause(id);
  itr->second.state = State::PAUSED;
}

void Executor::resume(const std::string & id)
{
  std::lock_guard lk(mtx_);
  auto itr = status_.find(id);
  if (itr == status_.end()) {
    throw exception::TaskExecutionException("Task %s not started, cannot resume.", id.c_str());
  }

  if (itr->second.state != State::PAUSED) {
    // ignore if the task is not at paused state
    return;
  }

  get_plugin<ExecutionInterface>(itr->second.runtime_interface)->resume(id);
  itr->second.state = State::ONGOING;
}

void Executor::cancel(const std::string & id)
{
  std::lock_guard lk(mtx_);
  auto itr = status_.find(id);
  if (itr == status_.end()) {
    throw exception::TaskExecutionException("Task %s not started, cannot cancel.", id.c_str());
  }

  if (itr->second.state != State::PAUSED &&
    itr->second.state != State::ONGOING)
  {
    // ignore if the task is not at paused state
    return;
  }

  get_plugin<ExecutionInterface>(itr->second.runtime_interface)->cancel(id);
  itr->second.state = State::CANCELLED;

  // return the future object too
  auto completion_observer = std::dynamic_pointer_cast<ExecutionStatusCompletionObserver>(
    runtime_status_completion_observer_);
  completion_observer->cancel_callback(id);
}

const Executor::Status & Executor::get_status(const std::string & id) const
{
  std::lock_guard lk(mtx_);
  return status_.at(id);
}

bool Executor::is_ongoing(const std::string & id) const
{
  std::lock_guard lk(mtx_);
  auto itr = status_.find(id);
  if (itr == status_.end()) {
    return false;
  }
  if (itr->second.state == State::ONGOING ||
    itr->second.state == State::PAUSED)
  {
    return true;
  }

  return false;
}

bool Executor::is_completed(const std::string & id) const
{
  std::lock_guard lk(mtx_);
  auto itr = status_.find(id);
  if (itr == status_.end()) {
    return false;
  }
  if (itr->second.state == State::COMPLETED ||
    itr->second.state == State::FAILED ||
    itr->second.state == State::CANCELLED)
  {
    return true;
  }

  return false;
}

const std::unordered_map<std::string, Executor::Status> & Executor::all_status() const
{
  std::lock_guard lk(mtx_);
  return status_;
}

}  // namespace task

}  // namespace rmf_scheduler
