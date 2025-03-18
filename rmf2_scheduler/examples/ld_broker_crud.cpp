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

#include <iostream>
#include <random>

#include "rmf2_scheduler/log.hpp"
#include "rmf2_scheduler/data/uuid.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"
#include "rmf2_scheduler/utils/url_utils.hpp"
#include "rmf2_scheduler/http/transport.hpp"
#include "rmf2_scheduler/storage/schedule_stream_ld_broker.hpp"
#include "rmf2_scheduler/cache/task_action.hpp"
#include "rmf2_scheduler/cache/process_action.hpp"

namespace rs = rmf2_scheduler;

using Process = rs::data::Process;
using Edge = rs::data::Edge;
using Task = rs::data::Task;
using Time = rs::data::Time;
using Duration = rs::data::Duration;

std::default_random_engine generator;

std::normal_distribution<double> t1_distribution(300.0, 10.0);  // forklift
std::normal_distribution<double> t2_distribution(60.0, 2.0);  // palletizer
std::normal_distribution<double> amr1_distribution(600.0, 30.0);  // amr
std::normal_distribution<double> amr1_distribution_other(600.0, 30.0);  // amr
std::normal_distribution<double> amr2_distribution(600.0, 15.0);  // amr

void generate_tasks(
  const Time & start_time,
  const std::string & status,
  rs::cache::ScheduleCache::Ptr & cache
)
{
  // Forklift to pickup
  Task::Ptr task1 = std::make_shared<Task>();
  task1->id = rs::data::gen_uuid();
  task1->type = "rmf2/go_to_place";
  task1->start_time = start_time;
  task1->status = status;
  task1->description = "Forklift 1 pickup";
  task1->duration = Duration::from_seconds(
    t1_distribution(generator)
  );
  task1->resource_id = "forklift_1";
  task1->task_details =
    R"({
    "start_location": "dock_1",
    "end_location": "palletizer_1"
  })"_json;

  // Palletization
  Task::Ptr task2 = std::make_shared<Task>();
  task2->id = rs::data::gen_uuid();
  task2->type = "ihi/palletization";
  task2->start_time = start_time;
  task2->status = status;
  task2->description = "Palletization";
  task2->duration = Duration::from_seconds(
    t2_distribution(generator)
  );
  task2->resource_id = "palletizer_1";
  task2->task_details = R"({
    "forklift_id": "forklift_1"
  })"_json;

  // Task 3 AMR task
  Task::Ptr task3 = std::make_shared<Task>();
  task3->id = rs::data::gen_uuid();
  task3->type = "rmf2/go_to_place";
  task3->start_time = start_time + Duration::from_seconds(360);
  task3->status = status;
  task3->description = "AMR1 transport task";
  task3->duration = Duration::from_seconds(
    amr1_distribution(generator)
  );
  task3->resource_id = "amr1";
  task3->task_details =
    R"({
    "start_location": "palletizer_1",
    "end_location": "drop1"
  })"_json;

  // Task 4 AMR task
  Task::Ptr task4 = std::make_shared<Task>();
  task4->id = rs::data::gen_uuid();
  task4->type = "rmf2/go_to_place";
  task4->start_time = start_time + Duration::from_seconds(400);
  task4->status = status;
  task4->description = "AMR2 transport task";
  task4->duration = Duration::from_seconds(
    amr2_distribution(generator)
  );
  task4->resource_id = "amr2";
  task4->task_details =
    R"({
    "start_location": "palletizer_1",
    "end_location": "drop1"
  })"_json;

  // Task 5 AMR task
  Task::Ptr task5 = std::make_shared<Task>();
  task5->id = rs::data::gen_uuid();
  task5->type = "rmf2/go_to_place";
  task5->start_time = start_time;
  task5->status = status;
  task5->description = "AMR2 transport task";
  task5->duration = Duration::from_seconds(
    amr1_distribution_other(generator)
  );
  task5->resource_id = "amr2";
  task5->task_details =
    R"({
    "start_location": "palletizer_1",
    "end_location": "drop1"
  })"_json;

  std::vector<Task::Ptr> tasks {
    task1, task2, task3, task4, task5
  };

  Process::Ptr process = std::make_unique<Process>();
  process->id = rs::data::gen_uuid();

  // Add all nodes
  for (auto & task : tasks) {
    process->graph.add_node(task->id);
  }

  process->graph.add_edge(task1->id, task2->id);
  process->graph.add_edge(task2->id, task3->id, Edge("soft"));
  process->graph.add_edge(task2->id, task4->id, Edge("soft"));
  process->graph.add_edge(task2->id, task5->id, Edge("soft"));
  process->graph.add_edge(task3->id, task5->id, Edge("hard"));

  std::string error;

  // Add tasks to the schedule cache
  for (auto & task : tasks) {
    rs::cache::TaskAction action(
      rs::data::action_type::TASK_ADD,
      rs::cache::ActionPayload().task(task)
    );
    bool result = action.validate(cache, error);
    if (!result) {
      std::cout << error << std::endl;
    }
    action.apply();
  }

  // Add process to the schedule cache
  rs::cache::ProcessAction process_action(
    rs::data::action_type::PROCESS_ADD,
    rs::cache::ActionPayload().process(process)
  );
  process_action.validate(cache, error);
  process_action.apply();

  // Update process start time
  rs::cache::ProcessAction process_update_start_time_action(
    rs::data::action_type::PROCESS_UPDATE_START_TIME,
    rs::cache::ActionPayload().id(process->id)
  );
  process_update_start_time_action.validate(cache, error);
  process_update_start_time_action.apply();
}

int main()
{
  rs::log::setLogLevel(rs::LogLevel::DEBUG);

  // Context broker URL
  std::string ld_broker_domain = "http://localhost:9090";
  std::string ld_broker_path = "ngsi-ld";

  // Create URL
  std::string ld_broker_url = rs::url::combine(
    ld_broker_domain,
    ld_broker_path
  );

  auto transport = rs::http::Transport::create_default();
  rs::storage::ScheduleStream::Ptr storage_stream =
    std::make_shared<rs::storage::ld_broker::ScheduleStream>(
    "http://localhost:9090/ngsi-ld",
    transport
    );

  // Write 100 times
  for (int i = 0; i < 1; i++) {
    // Create a cache first
    auto schedule_cache = std::make_shared<rs::cache::ScheduleCache>();
    std::string error;

    // Add the tasks to the cache
    generate_tasks(
      Time::from_ISOtime("2025-01-21T10:00:00Z") + Duration::from_seconds(7200 * i),
      "completed",
      schedule_cache
    );

    // Write to LD broker
    bool result = storage_stream->write_schedule(
      schedule_cache,
      rs::data::TimeWindow {
      rs::data::Time(0),
      rs::data::Time::max()
    },
      error
    );

    if (!result) {
      LOG_ERROR("%s", error.c_str());
      return 0;
    }
  }
}
