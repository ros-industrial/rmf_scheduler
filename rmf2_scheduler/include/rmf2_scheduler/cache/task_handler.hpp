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

#ifndef RMF2_SCHEDULER__CACHE__TASK_HANDLER_HPP_
#define RMF2_SCHEDULER__CACHE__TASK_HANDLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rmf2_scheduler/data/task.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace cache
{

class ScheduleCache;

class TaskHandler
{
public:
  RS_SMART_PTR_DEFINITIONS(TaskHandler)

  using TaskIterator = std::unordered_map<std::string, data::Task::Ptr>::iterator;
  using TaskConstIterator = std::unordered_map<std::string, data::Task::Ptr>::const_iterator;

  class Restricted;

  explicit TaskHandler(
    const std::shared_ptr<ScheduleCache> & cache,
    const Restricted &
  );

  virtual ~TaskHandler();

  TaskIterator emplace(const data::Task::Ptr & task);

  TaskIterator emplace(const data::Event::Ptr & event);

  void replace(
    const TaskIterator & itr,
    const data::Task::Ptr & task
  );

  void replace(
    const TaskConstIterator & itr,
    const data::Task::Ptr & task
  );

  void replace(
    const TaskIterator & itr,
    const data::Event::Ptr & event
  );

  void replace(
    const TaskConstIterator & itr,
    const data::Event::Ptr & event
  );

  void erase(
    const TaskIterator & itr
  );

  void erase(
    const TaskConstIterator & itr
  );

  bool find(
    const std::string & id,
    TaskIterator & itr
  );

  bool find(
    const std::string & id,
    TaskConstIterator & itr
  ) const;

private:
  RS_DISABLE_COPY(TaskHandler)

  void _add_task_time_window_lookup(
    const std::shared_ptr<ScheduleCache> & cache,
    const data::Task::Ptr & task
  );

  void _remove_task_time_window_lookup(
    const std::shared_ptr<ScheduleCache> & cache,
    const std::string & id
  );

  std::shared_ptr<ScheduleCache> _acquire_cache() const;

  std::weak_ptr<ScheduleCache> cache_wp_;
};

class TaskHandler::Restricted
{
private:
  Restricted();
  virtual ~Restricted();

  friend class ScheduleCache;
  friend class TestTaskHandler;
};

}  // namespace cache
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__CACHE__TASK_HANDLER_HPP_
