// Copyright 2025 ROS Industrial Consortium Asia Pacific
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

#include <chrono>
#include <iostream>
#include <random>

#include "rmf2_scheduler/cache/action.hpp"
#include "rmf2_scheduler/storage/schedule_stream_simple.hpp"

namespace rs = rmf2_scheduler;

bool write_schedule_event_only(
  const rs::storage::ScheduleStream::Ptr & schedule_stream,
  std::string & error
)
{
  rs::data::Task::Ptr task = rs::data::Task::make_shared();
  task->id = "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6";
  task->type = "task_type";
  task->description = "This is a task!";
  task->start_time = rs::data::Time::from_ISOtime("2025-01-22T02:15:00Z");
  task->duration = std::chrono::hours(1);
  task->resource_id = "09d5323d-43e0-4668-b53a-3cf33f9b9a96";
  task->deadline = rs::data::Time::from_ISOtime("2025-01-22T04:15:00Z");
  task->status = "ongoing";
  task->planned_start_time = rs::data::Time::from_ISOtime("2025-01-22T02:15:00Z");
  task->planned_duration = std::chrono::hours(1);
  task->estimated_duration = std::chrono::seconds(3700);
  task->actual_start_time = rs::data::Time::from_ISOtime("2025-01-22T02:16:00Z");
  task->actual_duration = std::chrono::seconds(3700);
  task->task_details = R"({"some_fields": "some_value"})"_json;

  auto schedule_cache = rs::cache::ScheduleCache::make_shared();
  auto task_add_action = rs::cache::Action::create(
    rs::data::action_type::TASK_ADD,
    rs::cache::ActionPayload().task(task)
  );

  if (!task_add_action->validate(schedule_cache, error)) {
    return false;
  }

  task_add_action->apply();

  rs::data::TimeWindow time_window;
  time_window.start = rs::data::Time::from_ISOtime("2025-01-22T02:14:00Z");
  time_window.end = rs::data::Time::from_ISOtime("2025-01-22T02:16:00Z");

  return schedule_stream->write_schedule(schedule_cache, time_window, error);
}

bool write_schedule_event_and_process(
  const rs::storage::ScheduleStream::Ptr & schedule_stream,
  std::string & error
)
{
  rs::data::Task::Ptr task1 = rs::data::Task::make_shared();
  task1->id = "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6";
  task1->type = "task_type";
  task1->description = "This is a task!";
  task1->start_time = rs::data::Time::from_ISOtime("2025-01-22T02:15:00Z");
  task1->status = "ongoing";

  rs::data::Task::Ptr task2 = rs::data::Task::make_shared();
  task2->id = "8a9f4819-bbd4-4550-acce-e061d3289973";
  task2->type = "task_type";
  task2->description = "This is a task!";
  task2->start_time = rs::data::Time::from_ISOtime("2025-01-22T04:15:00Z");
  task2->status = "ongoing";

  rs::data::Process::Ptr process = rs::data::Process::make_shared();
  process->id = "13aa1c62-64ca-495d-a4b7-84de6a00f56a";
  process->graph.add_node("8a9f4819-bbd4-4550-acce-e061d3289973");
  process->graph.add_node("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  process->graph.add_edge(
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "8a9f4819-bbd4-4550-acce-e061d3289973",
    rs::data::Edge("hard")
  );

  auto schedule_cache = rs::cache::ScheduleCache::make_shared();
  std::vector<rs::data::Task::Ptr> tasks {task1, task2};
  std::vector<rs::data::Process::Ptr> processes {process};

  for (auto & task : tasks) {
    auto task_add_action = rs::cache::Action::create(
      rs::data::action_type::TASK_ADD,
      rs::cache::ActionPayload().task(task)
    );

    if (!task_add_action->validate(schedule_cache, error)) {
      return false;
    }

    task_add_action->apply();
  }

  for (auto & process : processes) {
    auto process_add_action = rs::cache::Action::create(
      rs::data::action_type::PROCESS_ADD,
      rs::cache::ActionPayload().process(process)
    );

    if (!process_add_action->validate(schedule_cache, error)) {
      return false;
    }

    process_add_action->apply();
  }

  rs::data::TimeWindow time_window;
  time_window.start = rs::data::Time::from_ISOtime("2025-01-22T02:14:00Z");
  time_window.end = rs::data::Time::from_ISOtime("2025-01-22T04:16:00Z");

  return schedule_stream->write_schedule(schedule_cache, time_window, error);
}

int main()
{
  // event only example backup
  rs::storage::ScheduleStream::Ptr storage_stream_event_only =
    std::make_shared<rs::storage::simple::ScheduleStream>(
    5,
    "~/.cache/sample_r2ts_backups/event_only"
    );

  std::string error;
  bool result = write_schedule_event_only(storage_stream_event_only, error);
  if (!result) {
    std::cerr << error << std::endl;
    return 1;
  }

  // event and process example backup
  rs::storage::ScheduleStream::Ptr storage_stream_event_and_process =
    std::make_shared<rs::storage::simple::ScheduleStream>(
    5,
    "~/.cache/sample_r2ts_backups/event_and_process"
    );

  result = write_schedule_event_and_process(storage_stream_event_and_process, error);
  if (!result) {
    std::cerr << error << std::endl;
    return 1;
  }
}
