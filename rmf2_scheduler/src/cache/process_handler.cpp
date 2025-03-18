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
#include "rmf2_scheduler/utils/dag_helper.hpp"
#include "rmf2_scheduler/cache/schedule_cache.hpp"
#include "rmf2_scheduler/cache/process_handler.hpp"

namespace rmf2_scheduler
{

namespace cache
{

ProcessHandler::ProcessHandler(
  const ScheduleCache::Ptr & cache,
  const Restricted &
)
: cache_wp_(cache)
{
}

ProcessHandler::~ProcessHandler()
{
}

ProcessHandler::ProcessIterator ProcessHandler::emplace(
  const data::Process::Ptr & process
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  auto result = cache->process_map.emplace(process->id, process);
  return result.first;
}

void ProcessHandler::replace(
  const ProcessIterator & itr,
  const data::Process::Ptr & process
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  // Update process in the list
  itr->second = process;
}

void ProcessHandler::replace(
  const ProcessConstIterator & itr_const,
  const data::Process::Ptr & process
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  ProcessIterator itr = utils::remove_iterator_const(cache->process_map, itr_const);

  replace(itr, process);
}

void ProcessHandler::erase(
  const ProcessIterator & itr
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  cache->process_map.erase(itr);
}

void ProcessHandler::erase(
  const ProcessConstIterator & itr_const
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  ProcessIterator itr = utils::remove_iterator_const(cache->process_map, itr_const);

  erase(itr);
}

void ProcessHandler::update_start_time(
  const ProcessIterator & itr
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  utils::update_dag_start_time(itr->second->graph, cache->task_map);
}

void ProcessHandler::update_start_time(
  const ProcessConstIterator & itr_const
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  ProcessIterator itr = utils::remove_iterator_const(cache->process_map, itr_const);

  update_start_time(itr);
}

bool ProcessHandler::find(
  const std::string & id,
  ProcessIterator & itr
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  itr = cache->process_map.find(id);
  return itr != cache->process_map.end();
}

bool ProcessHandler::find(
  const std::string & id,
  ProcessConstIterator & itr
) const
{
  ScheduleCache::Ptr cache = _acquire_cache();

  itr = cache->process_map.find(id);
  return itr != cache->process_map.end();
}

ScheduleCache::Ptr ProcessHandler::_acquire_cache() const
{
  ScheduleCache::Ptr cache;
  if (!(cache = cache_wp_.lock())) {
    throw std::runtime_error("ScheduleCache has expired");
  }
  return cache;
}

ProcessHandler::Restricted::Restricted()
{
}

ProcessHandler::Restricted::~Restricted()
{
}

}  // namespace cache
}  // namespace rmf2_scheduler
