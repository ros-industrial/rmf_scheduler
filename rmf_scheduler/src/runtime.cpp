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

#include "rmf_scheduler/runtime.hpp"

namespace rmf_scheduler
{

class RuntimeStatusCompletionObserver : public RuntimeObserverBase
{
public:
  RuntimeStatusCompletionObserver(
    std::mutex & status_mtx,
    std::unordered_map<std::string, Runtime::Status> & status)
  : mtx_(status_mtx), status_(status)
  {
  }

  std::shared_future<Runtime::Status> get_future(const std::string & id)
  {
    ongoing_task_sig_[id] = std::promise<Runtime::Status>();
    return ongoing_task_sig_.at(id).get_future();
  }

  void completion_callback(
    const std::string & id,
    bool success,
    const std::string & detail = "") final
  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto & status = status_[id];
    status = Runtime::Status {
      success ? Runtime::State::COMPLETED : Runtime::State::FAILED,
      "",
      detail
    };
    auto itr = ongoing_task_sig_.find(id);
    itr->second.set_value(status);
    ongoing_task_sig_.erase(itr);
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
  std::unordered_map<std::string, Runtime::Status> & status_;
  std::unordered_map<std::string, std::promise<Runtime::Status>> ongoing_task_sig_;
};

Runtime::Runtime()
{
  interface_loader_ = std::make_shared<pluginlib::ClassLoader<RuntimeInterfaceBase>>(
    "rmf_scheduler", "rmf_scheduler::RuntimeInterfaceBase");

  runtime_status_completion_observer_ = make_observer<RuntimeStatusCompletionObserver>(
    mtx_, status_);
}

void Runtime::load_runtime_interface(
  const std::string & interface,
  const std::string & name,
  const std::string & ns)
{
  if (name.empty()) {
    throw RuntimeException("New runtime interface name cannot be empty.");
  }

  std::string full_name = ns.empty() ? name : ns + "/" + name;

  if (interfaces_.find(full_name) != interfaces_.end()) {
    throw RuntimeException("New runtime interface %s already exists.", full_name.c_str());
  }

  auto interface_instance = interface_loader_->createUnmanagedInstance(interface);

  auto supported_task_types = interface_instance->support_task_types();

  for (auto & supported_task : supported_task_types) {
    // Check if an existing interface can execute the same supported task
    auto itr = task_runtime_interface_lookup_.find(supported_task);
    if (itr != task_runtime_interface_lookup_.end()) {
      throw RuntimeException(
              "New runtime interface %s [%s] shares the execution of the same task type [%s]"
              "with %s, please consider removing %s before adding %s.",
              full_name.c_str(), interface.c_str(),
              itr->first.c_str(),
              itr->second.c_str(),
              itr->second.c_str(), full_name.c_str());
    }
  }

  interface_instance->init(name, ns);

  for (auto observer : observers_) {
    interface_instance->attach(observer);
  }

  interfaces_.emplace(full_name, interface_instance);

  // Add lookup for the runtime interface for each task
  for (auto & supported_task : supported_task_types) {
    task_runtime_interface_lookup_[supported_task] = full_name;
  }
}

void Runtime::unload_runtime_interface(
  const std::string & name,
  const std::string & ns)
{
  std::string full_name = ns.empty() ? name : ns + "/" + name;
  interfaces_.erase(full_name);

  // Erase lookup entry too
  for (auto entry = task_runtime_interface_lookup_.begin();
    entry != task_runtime_interface_lookup_.end(); entry++)
  {
    if (entry->second == full_name) {
      task_runtime_interface_lookup_.erase(entry);
    }
  }
}

std::shared_future<Runtime::Status> Runtime::run_async(
  const Event & event)
{
  auto itr = task_runtime_interface_lookup_.find(event.type);
  if (itr != task_runtime_interface_lookup_.end()) {
    throw RuntimeException(
            "No runtime interface configured for task type [%s]",
            event.type.c_str());
  }

  std::lock_guard lk(mtx_);
  interfaces_.at(itr->second)->start(event.id, event.event_details);
  status_[event.id] = Status{State::ONGOING, itr->second, ""};
  auto completion_observer = std::dynamic_pointer_cast<RuntimeStatusCompletionObserver>(
    runtime_status_completion_observer_);

  return completion_observer->get_future(event.id);
}

void Runtime::pause(const std::string & id)
{
  std::lock_guard lk(mtx_);
  auto itr = status_.find(id);
  if (itr == status_.end()) {
    throw RuntimeException("Task %s not started, cannot pause.", id.c_str());
  }

  if (itr->second.state != State::ONGOING) {
    // ignore if the task is not ongoing
    return;
  }

  interfaces_.at(itr->second.runtime_interface)->pause(id);
  itr->second.state = State::PAUSED;
}

void Runtime::resume(const std::string & id)
{
  std::lock_guard lk(mtx_);
  auto itr = status_.find(id);
  if (itr == status_.end()) {
    throw RuntimeException("Task %s not started, cannot resume.", id.c_str());
  }

  if (itr->second.state != State::PAUSED) {
    // ignore if the task is not at paused state
    return;
  }

  interfaces_.at(itr->second.runtime_interface)->resume(id);
  itr->second.state = State::ONGOING;
}

void Runtime::cancel(const std::string & id)
{
  std::lock_guard lk(mtx_);
  auto itr = status_.find(id);
  if (itr == status_.end()) {
    throw RuntimeException("Task %s not started, cannot cancel.", id.c_str());
  }

  if (itr->second.state != State::PAUSED &&
    itr->second.state != State::ONGOING)
  {
    // ignore if the task is not at paused state
    return;
  }

  interfaces_.at(itr->second.runtime_interface)->cancel(id);
  itr->second.state = State::CANCELLED;

  // return the future object too
  auto completion_observer = std::dynamic_pointer_cast<RuntimeStatusCompletionObserver>(
    runtime_status_completion_observer_);
  completion_observer->cancel_callback(id);
}

const std::unordered_map<std::string, std::string> & Runtime::supported_task_interface() const
{
  return task_runtime_interface_lookup_;
}

const Runtime::Status & Runtime::get_status(const std::string & id) const
{
  std::lock_guard lk(mtx_);
  return status_.at(id);
}


const std::unordered_map<std::string, Runtime::Status> & Runtime::all_status() const
{
  std::lock_guard lk(mtx_);
  return status_;
}

}  // namespace rmf_scheduler
