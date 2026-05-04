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
#include "rmf2_scheduler/cache/series_handler.hpp"

namespace rmf2_scheduler
{

namespace cache
{

SeriesHandler::SeriesHandler(
  const ScheduleCache::Ptr & cache,
  const Restricted &
)
: cache_wp_(cache)
{
}

SeriesHandler::~SeriesHandler()
{
}

SeriesHandler::SeriesIterator SeriesHandler::emplace(
  const data::Series::Ptr & series
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  auto result = cache->series_map.emplace(series->id(), series);
  return result.first;
}

void SeriesHandler::replace(
  const SeriesHandler::SeriesIterator & itr,
  const data::Series::Ptr & series
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  // Update series in the list
  itr->second = series;
}

void SeriesHandler::replace(
  const SeriesConstIterator & itr_const,
  const data::Series::Ptr & series
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  SeriesIterator itr = utils::remove_iterator_const(cache->series_map, itr_const);

  replace(itr, series);
}

void SeriesHandler::erase(
  const SeriesIterator & itr
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  cache->series_map.erase(itr);
}

void SeriesHandler::erase(
  const SeriesConstIterator & itr_const
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  SeriesIterator itr = utils::remove_iterator_const(cache->series_map, itr_const);

  erase(itr);
}

bool SeriesHandler::find(
  const std::string & id,
  SeriesIterator & itr
)
{
  ScheduleCache::Ptr cache = _acquire_cache();

  itr = cache->series_map.find(id);
  return itr != cache->series_map.end();
}

bool SeriesHandler::find(
  const std::string & id,
  SeriesConstIterator & itr
) const
{
  ScheduleCache::Ptr cache = _acquire_cache();

  itr = cache->series_map.find(id);
  return itr != cache->series_map.end();
}

ScheduleCache::Ptr SeriesHandler::_acquire_cache() const
{
  ScheduleCache::Ptr cache;
  if (!(cache = cache_wp_.lock())) {
    throw std::runtime_error("ScheduleCache has expired");
  }
  return cache;
}

SeriesHandler::Restricted::Restricted()
{
}

SeriesHandler::Restricted::~Restricted()
{
}

}  // namespace cache
}  // namespace rmf2_scheduler
