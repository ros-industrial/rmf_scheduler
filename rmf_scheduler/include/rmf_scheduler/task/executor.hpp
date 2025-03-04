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

#ifndef RMF_SCHEDULER__TASK__EXECUTOR_HPP_
#define RMF_SCHEDULER__TASK__EXECUTOR_HPP_

#include <future>
#include <memory>
#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rmf_scheduler/task/execution_interface.hpp"
#include "rmf_scheduler/data/event.hpp"

namespace rmf_scheduler
{

namespace task
{

class Executor : public TaskPluginManager
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

  Executor();
  virtual ~Executor() {}

  void load_plugin(
    const std::shared_ptr<void> & node,
    const std::string & name,
    const std::string & interface,
    const std::vector<std::string> & supported_tasks) override;

  void unload_plugin(
    const std::string & name) override;

  template<typename Observer, typename ... ArgsT>
  std::shared_ptr<Observer> make_observer(ArgsT &&... args);

  template<typename Observer>
  void remove_observer(std::shared_ptr<Observer> observer);

  std::shared_future<Status> run_async(
    const data::Event & event);

  void pause(const std::string & id);

  void resume(const std::string & id);

  void cancel(const std::string & id);

  const Status & get_status(const std::string &) const;

  bool is_ongoing(const std::string &) const;
  bool is_completed(const std::string &) const;

  const std::unordered_map<std::string, Status> & all_status() const;

private:
  // TODO(Bainian): mutex might not needed here
  mutable std::mutex mtx_;
  std::unordered_map<std::string, Status> status_;

  std::unordered_set<std::shared_ptr<ExecutionObserverBase>> observers_;

  std::shared_ptr<ExecutionObserverBase> runtime_status_completion_observer_;
};

template<typename ObserverT, typename ... ArgsT>
std::shared_ptr<ObserverT> Executor::make_observer(ArgsT &&... args)
{
  static_assert(
    std::is_base_of_v<ExecutionObserverBase, ObserverT>,
    "Observer must be derived from RuntimeInterfaceBase"
  );

  // use a local variable to mimic the constructor
  auto ptr = std::make_shared<ObserverT>(std::forward<ArgsT>(args)...);
  observers_.emplace(std::static_pointer_cast<ExecutionObserverBase>(ptr));

  for_each_plugin<ExecutionInterface>(
    [ptr](
      const std::string &, std::shared_ptr<ExecutionInterface> interface) -> void
    {
      interface->attach(std::static_pointer_cast<ExecutionObserverBase>(ptr));
    });
  return ptr;
}

template<typename ObserverT>
void Executor::remove_observer(std::shared_ptr<ObserverT> ptr)
{
  static_assert(
    std::is_base_of_v<ExecutionObserverBase, ObserverT>,
    "Observer must be derived from RuntimeObserverBase"
  );

  observers_.erase(std::static_pointer_cast<ExecutionObserverBase>(ptr));

  for_each_plugin<ExecutionInterface>(
    [ptr](
      const std::string &, std::shared_ptr<ExecutionInterface> interface) -> void
    {
      interface->detach(std::static_pointer_cast<ExecutionObserverBase>(ptr));
    });
}

}  // namespace task

namespace exception
{

class TaskExecutionException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  TaskExecutionException(const char * msg, Args && ... args)
  : ExceptionTemplate(
      ErrorCode::FAILURE | ErrorCode::RUNTIME | ErrorCode::NO_FIELD,
      msg, std::forward<Args>(args) ...)
  {
    add_prefix("TaskExecutionException:\n  ");
  }
};

}  // namespace exception

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__TASK__EXECUTOR_HPP_
