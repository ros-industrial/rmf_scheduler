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

#ifndef RMF_SCHEDULER__RUNTIME_HPP_
#define RMF_SCHEDULER__RUNTIME_HPP_

#include <future>
#include <memory>
#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>

#include "pluginlib/class_loader.hpp"

#include "rmf_scheduler/runtime_interface.hpp"
#include "rmf_scheduler/event.hpp"
#include "rmf_scheduler/system_time_executor.hpp"

namespace rmf_scheduler
{

class Runtime
{
public:
  enum class State : std::uint8_t
  {
    QUEUED = 0,
    ONGOING = 1,
    PAUSED = 2,
    COMPLETED = 3,
    FAILED = 4,
    CANCELLED = 5,
  };

  struct Status
  {
    State state;
    std::string runtime_interface;
    std::string detail;
  };

  Runtime();
  virtual ~Runtime() {}

  void load_runtime_interface(
    const std::string & interface,
    const std::string & name,
    const std::string & ns = "");

  void unload_runtime_interface(
    const std::string & name,
    const std::string & ns = "");

  template<typename Observer, typename ... ArgsT>
  std::shared_ptr<Observer> make_observer(ArgsT &&... args);

  template<typename Observer>
  void remove_observer(std::shared_ptr<Observer> observer);

  std::shared_future<Status> run_async(
    const Event & event);

  void pause(const std::string & id);

  void resume(const std::string & id);

  void cancel(const std::string & id);

  const std::unordered_map<std::string, std::string> & supported_task_interface() const;

  const Status & get_status(const std::string &) const;

  const std::unordered_map<std::string, Status> & all_status() const;

private:
  // TODO(Bainian): mutex might not needed here
  mutable std::mutex mtx_;
  std::unordered_map<std::string, Status> status_;

  std::unordered_map<std::string, std::string> task_runtime_interface_lookup_;
  std::unordered_map<std::string, std::unique_ptr<RuntimeInterfaceBase>> interfaces_;

  std::shared_ptr<pluginlib::ClassLoader<RuntimeInterfaceBase>> interface_loader_;
  std::unordered_set<std::shared_ptr<RuntimeObserverBase>> observers_;

  std::shared_ptr<RuntimeObserverBase> runtime_status_completion_observer_;
};


class RuntimeException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  RuntimeException(const char * msg, Args && ... args)
  : ExceptionTemplate(msg, std::forward<Args>(args) ...)
  {
    add_prefix("RuntimeInitializationException:\n  ");
  }
};


template<typename Observer, typename ... ArgsT>
std::shared_ptr<Observer> Runtime::make_observer(ArgsT &&... args)
{
  static_assert(
    std::is_base_of_v<RuntimeObserverBase, Observer>,
    "Observer must be derived from RuntimeInterfaceBase"
  );

  // use a local variable to mimic the constructor
  auto ptr = std::make_shared<Observer>(std::forward<ArgsT>(args)...);

  observers_.emplace(std::static_pointer_cast<RuntimeObserverBase>(ptr));
  for (auto & interface : interfaces_) {
    interface.second->attach(std::static_pointer_cast<RuntimeObserverBase>(ptr));
  }
  return ptr;
}

template<typename Observer>
void Runtime::remove_observer(std::shared_ptr<Observer> ptr)
{
  static_assert(
    std::is_base_of_v<RuntimeObserverBase, Observer>,
    "Observer must be derived from RuntimeObserverBase"
  );

  observers_.erase(std::static_pointer_cast<RuntimeObserverBase>(ptr));
  for (auto & interface : interfaces_) {
    interface.second->detach(std::static_pointer_cast<RuntimeObserverBase>(ptr));
  }
}

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__RUNTIME_HPP_
