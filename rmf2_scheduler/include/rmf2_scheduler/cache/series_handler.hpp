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

#ifndef RMF2_SCHEDULER__CACHE__SERIES_HANDLER_HPP_
#define RMF2_SCHEDULER__CACHE__SERIES_HANDLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rmf2_scheduler/data/series.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace cache
{

class ScheduleCache;

class SeriesHandler
{
public:
  RS_SMART_PTR_DEFINITIONS(SeriesHandler)

  using SeriesIterator = std::unordered_map<std::string, data::Series::Ptr>::iterator;
  using SeriesConstIterator = std::unordered_map<std::string, data::Series::Ptr>::const_iterator;

  class Restricted;

  explicit SeriesHandler(
    const std::shared_ptr<ScheduleCache> & cache,
    const Restricted &
  );

  virtual ~SeriesHandler();

  SeriesIterator emplace(const data::Series::Ptr & series);

  void replace(
    const SeriesIterator & itr,
    const data::Series::Ptr & series
  );

  void replace(
    const SeriesConstIterator & itr,
    const data::Series::Ptr & series
  );

  void erase(
    const SeriesIterator & itr
  );

  void erase(
    const SeriesConstIterator & itr
  );

  bool find(
    const std::string & id,
    SeriesIterator & itr
  );

  bool find(
    const std::string & id,
    SeriesConstIterator & itr
  ) const;

private:
  RS_DISABLE_COPY(SeriesHandler)

  std::shared_ptr<ScheduleCache> _acquire_cache() const;

  std::weak_ptr<ScheduleCache> cache_wp_;
};

class SeriesHandler::Restricted
{
private:
  Restricted();
  virtual ~Restricted();

  friend class ScheduleCache;
  friend class TestSeriesHandler;
};

}  // namespace cache
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__CACHE__SERIES_HANDLER_HPP_
