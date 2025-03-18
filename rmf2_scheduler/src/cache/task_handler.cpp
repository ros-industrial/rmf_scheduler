// Copyright 2024-2025 ROS Industrial Consortium Asia Pacific
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

#include "rmf2_scheduler/utils/utils.hpp"
#include "rmf2_scheduler/cache/schedule_cache.hpp"
#include "rmf2_scheduler/cache/task_handler.hpp"

namespace rmf2_scheduler
{

namespace cache
{

TaskHandler::TaskHandler(
  const ScheduleCache::Ptr & cache,
  const Restricted &
)
: cache_wp_(cache)
{
}

TaskHandler::~TaskHandler()
{
}

TaskHandler::TaskIterator TaskHandler::emplace(
  const data::Task::Ptr & task
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  _add_task_time_window_lookup(cache, task);
  auto result = cache->task_map.emplace(task->id, task);
  return result.first;
}

TaskHandler::TaskIterator TaskHandler::emplace(
  const data::Event::Ptr & event
)
{
  return emplace(data::Task::make_shared(*event, ""));
}

void TaskHandler::replace(
  const TaskIterator & itr,
  const data::Task::Ptr & task
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  // Update time window lookup table if tasks start time is changed
  if (itr->second->start_time != task->start_time ||
    itr->second->duration != task->duration)
  {
    _remove_task_time_window_lookup(cache, task->id);
    _add_task_time_window_lookup(cache, task);
  }

  // Update task in the list
  itr->second = task;
}

void TaskHandler::replace(
  const TaskConstIterator & itr_const,
  const data::Task::Ptr & task
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  TaskIterator itr = utils::remove_iterator_const(cache->task_map, itr_const);

  replace(itr, task);
}

void TaskHandler::replace(
  const TaskIterator & itr,
  const data::Event::Ptr & event
)
{
  replace(itr, data::Task::make_shared(*event, ""));
}

void TaskHandler::replace(
  const TaskConstIterator & itr,
  const data::Event::Ptr & event
)
{
  replace(itr, data::Task::make_shared(*event, ""));
}

void TaskHandler::erase(
  const TaskIterator & itr
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  _remove_task_time_window_lookup(cache, itr->second->id);
  cache->task_map.erase(itr);
}

void TaskHandler::erase(
  const TaskConstIterator & itr_const
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  TaskIterator itr = utils::remove_iterator_const(cache->task_map, itr_const);

  erase(itr);
}

bool TaskHandler::find(
  const std::string & id,
  TaskIterator & itr
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  itr = cache->task_map.find(id);
  return itr != cache->task_map.end();
}

bool TaskHandler::find(
  const std::string & id,
  TaskConstIterator & itr
) const
{
  ScheduleCache::Ptr cache = _acquire_cache();

  itr = cache->task_map.find(id);
  return itr != cache->task_map.end();
}

void TaskHandler::_add_task_time_window_lookup(
  const std::shared_ptr<ScheduleCache> & cache,
  const data::Task::Ptr & task
)
{
  cache->time_window_lookup.add_entry(
      {
        task->id,
        "task",
        {
          task->start_time,
          task->start_time + task->duration.value_or(data::Duration(0, 0))
        }
      }
  );
}

void TaskHandler::_remove_task_time_window_lookup(
  const std::shared_ptr<ScheduleCache> & cache,
  const std::string & id
)
{
  cache->time_window_lookup.remove_entry(id);
}

ScheduleCache::Ptr TaskHandler::_acquire_cache() const
{
  ScheduleCache::Ptr cache;
  if (!(cache = cache_wp_.lock())) {
    throw std::runtime_error("ScheduleCache has expired");
  }
  return cache;
}

TaskHandler::Restricted::Restricted()
{
}

TaskHandler::Restricted::~Restricted()
{
}

}  // namespace cache
}  // namespace rmf2_scheduler
