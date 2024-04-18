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

#include "gtest/gtest.h"
#include "rmf_scheduler/cache.hpp"
#include "rmf_scheduler/utils/system_time_utils.hpp"
#include "rmf_scheduler/scheduler.hpp"

TEST(TestCache, InitializeCache)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)
  std::shared_ptr<Cache> cache_ptr_;
  std::string cache_dir = std::string(TEST_DIRECTORY) + "/cache";

  cache_ptr_ = std::make_shared<Cache>(5, cache_dir);

  data::Schedule schedule;
  cache_ptr_->from_last_cache(schedule);

  schedule = data::Schedule();
  // Add some schedule
  data::Event event1, event2;
  event1.id = "event1";
  event1.type = "default/robot_task";
  event1.start_time = 1680961268 * 1e9;
  event1.duration = 100 * 1e9;
  event1.dag_id = "dag-1";

  event2.id = "event2";
  event2.type = "default/robot_task";
  event2.start_time = 1680961268 * 1e9;
  event2.duration = 100 * 1e9;
  event2.dag_id = "dag-1";

  schedule.add_event(event1);
  schedule.add_event(event2);

  data::DAG dag;
  dag.add_node("event1");
  dag.add_node("event2");
  dag.add_dependency("event1", {"event2"});

  schedule.add_dag("dag-1", std::move(dag));

  data::Series series(
    "dag-1",
    1680961268 * 1e9,
    "08 41 21 ? * Sat,Sun",
    "Asia/Singapore");

  schedule.add_dag_series("series-1", std::move(series));

  cache_ptr_->write_local_cache(schedule);

  schedule.expand_all_series_until(utils::now());
  cache_ptr_->write_local_cache(schedule);
}
