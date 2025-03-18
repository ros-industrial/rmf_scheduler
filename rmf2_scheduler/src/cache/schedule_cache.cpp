// Copyright 2023-2024 ROS Industrial Consortium Asia Pacific
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

#include <iterator>

#include "rmf2_scheduler/cache/schedule_cache.hpp"
#include "rmf2_scheduler/cache/task_handler.hpp"
#include "rmf2_scheduler/cache/process_handler.hpp"
#include "rmf2_scheduler/utils/dag_helper.hpp"

namespace rmf2_scheduler
{

namespace cache
{

ScheduleCache::ScheduleCache(
)
{
}

ScheduleCache::~ScheduleCache()
{
}

// EVENT
data::Event::ConstPtr ScheduleCache::get_event(const std::string & id) const
{
  auto event_itr = task_map.find(id);
  if (event_itr == task_map.end()) {
    throw std::invalid_argument(
            "ScheduleCache get_event failed, event [" + id + "] doesn't exist."
    );
  }

  return event_itr->second;
}

std::vector<data::Event::ConstPtr> ScheduleCache::get_all_events() const
{
  std::vector<data::Event::ConstPtr> result;
  result.reserve(task_map.size());

  for (auto & itr : task_map) {
    result.push_back(itr.second);
  }

  return result;
}

bool ScheduleCache::has_event(const std::string & id) const
{
  return task_map.find(id) != task_map.end();
}

std::vector<data::Event::ConstPtr> ScheduleCache::lookup_events(
  const data::Time & start_time,
  const data::Time & end_time,
  bool soft_lower_bound,
  bool soft_upper_bound
) const
{
  auto tasks = lookup_tasks(
    start_time,
    end_time,
    soft_lower_bound,
    soft_upper_bound
  );

  return {tasks.begin(), tasks.end()};
}

size_t ScheduleCache::event_size() const
{
  return task_map.size();
}

// TASK
data::Task::ConstPtr ScheduleCache::get_task(const std::string & id) const
{
  auto task_itr = task_map.find(id);
  if (task_itr == task_map.end()) {
    throw std::invalid_argument(
            "ScheduleCache get_task failed, event [" + id + "] doesn't exist."
    );
  }
  return task_itr->second;
}

std::vector<data::Task::ConstPtr> ScheduleCache::get_all_tasks() const
{
  std::vector<data::Task::ConstPtr> result;
  result.reserve(task_map.size());

  for (auto & itr : task_map) {
    result.push_back(itr.second);
  }

  return result;
}

bool ScheduleCache::has_task(const std::string & id) const
{
  auto task_itr = task_map.find(id);
  return task_itr != task_map.end();
}

std::vector<data::Task::ConstPtr> ScheduleCache::lookup_tasks(
  const data::Time & start_time,
  const data::Time & end_time,
  bool soft_lower_bound,
  bool soft_upper_bound
) const
{
  auto entries = time_window_lookup.lookup(
    {start_time, end_time},
    soft_lower_bound,
    soft_upper_bound
  );

  std::vector<data::Task::ConstPtr> result;
  result.reserve(entries.size());
  for (auto & entry : entries) {
    if (entry.type != "task") {
      continue;
    }

    result.push_back(task_map.at(entry.id));
  }

  result.shrink_to_fit();
  return result;
}

size_t ScheduleCache::task_size() const
{
  return task_map.size();
}


// PROCESS
data::Process::ConstPtr ScheduleCache::get_process(const std::string & id) const
{
  auto process_itr = process_map.find(id);
  if (process_itr == process_map.end()) {
    throw std::invalid_argument(
            "ScheduleCache get_process failed, process [" + id + "] doesn't exist."
    );
  }

  return process_itr->second;
}

std::vector<data::Process::ConstPtr> ScheduleCache::get_all_processes() const
{
  std::vector<data::Process::ConstPtr> result;
  result.reserve(process_map.size());

  for (auto & itr : process_map) {
    result.push_back(itr.second);
  }

  return result;
}

bool ScheduleCache::has_process(const std::string & id) const
{
  return process_map.find(id) != process_map.end();
}

size_t ScheduleCache::process_size() const
{
  return process_map.size();
}

// UTILITIES
ScheduleCache::Ptr ScheduleCache::clone() const
{
  auto result = std::make_shared<ScheduleCache>();

  // Deep copy events and tasks
  for (auto itr : this->task_map) {
    result->task_map[itr.first] = std::make_shared<data::Task>(*itr.second);
  }

  // Deep copy process
  for (auto itr : this->process_map) {
    result->process_map[itr.first] = std::make_shared<data::Process>(*itr.second);
  }

  // Deep copy series
  for (auto itr : this->series_map) {
    result->series_map[itr.first] = std::make_shared<data::Series>(*itr.second);
  }

  // Deep copy lookup table
  result->time_window_lookup = this->time_window_lookup;
  return result;
}

std::shared_ptr<TaskHandler> ScheduleCache::make_task_handler(const TaskRestricted &)
{
  TaskHandler::Restricted token;
  return TaskHandler::make_shared(shared_from_this(), token);
}

std::shared_ptr<ProcessHandler> ScheduleCache::make_process_handler(const ProcessRestricted &)
{
  ProcessHandler::Restricted token;
  return ProcessHandler::make_shared(shared_from_this(), token);
}

// TOKEN CLASS
ScheduleCache::TaskRestricted::TaskRestricted()
{
}

ScheduleCache::TaskRestricted::~TaskRestricted()
{
}

ScheduleCache::ProcessRestricted::ProcessRestricted()
{
}

ScheduleCache::ProcessRestricted::~ProcessRestricted()
{
}

}  // namespace cache
}  // namespace rmf2_scheduler
