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

#ifndef RMF2_SCHEDULER__CACHE__PROCESS_HANDLER_HPP_
#define RMF2_SCHEDULER__CACHE__PROCESS_HANDLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rmf2_scheduler/data/process.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace cache
{

class ScheduleCache;

class ProcessHandler
{
public:
  RS_SMART_PTR_DEFINITIONS(ProcessHandler)

  using ProcessIterator = std::unordered_map<std::string, data::Process::Ptr>::iterator;
  using ProcessConstIterator = std::unordered_map<std::string, data::Process::Ptr>::const_iterator;

  class Restricted;

  explicit ProcessHandler(
    const std::shared_ptr<ScheduleCache> & cache,
    const Restricted &
  );

  virtual ~ProcessHandler();

  ProcessIterator emplace(const data::Process::Ptr & process);

  void replace(
    const ProcessIterator & itr,
    const data::Process::Ptr & process
  );

  void replace(
    const ProcessConstIterator & itr,
    const data::Process::Ptr & process
  );

  void erase(
    const ProcessIterator & itr
  );

  void erase(
    const ProcessConstIterator & itr
  );

  void update_start_time(
    const ProcessIterator & itr
  );

  void update_start_time(
    const ProcessConstIterator & itr
  );

  bool find(
    const std::string & id,
    ProcessIterator & itr
  );

  bool find(
    const std::string & id,
    ProcessConstIterator & itr
  ) const;

private:
  RS_DISABLE_COPY(ProcessHandler)

  std::shared_ptr<ScheduleCache> _acquire_cache() const;

  std::weak_ptr<ScheduleCache> cache_wp_;
};

class ProcessHandler::Restricted
{
private:
  Restricted();
  virtual ~Restricted();

  friend class ScheduleCache;
  friend class TestProcessHandler;
};

}  // namespace cache
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__CACHE__PROCESS_HANDLER_HPP_
