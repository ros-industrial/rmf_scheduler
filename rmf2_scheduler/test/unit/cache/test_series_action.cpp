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

#include <cmath>
#include "gtest/gtest.h"

#include "rmf2_scheduler/cache/event_action.hpp"
#include "rmf2_scheduler/cache/series_action.hpp"
#include "rmf2_scheduler/cache/process_action.hpp"
#include "rmf2_scheduler/cache/schedule_cache.hpp"
#include "../../utils/gtest_macros.hpp"

namespace rmf2_scheduler
{

namespace cache
{

class TestScheduleCache
{
public:
  static rmf2_scheduler::cache::ScheduleCache::UPtr create_empty_cache()
  {
    return std::make_unique<ScheduleCache>();
  }

  static rmf2_scheduler::cache::ScheduleCache::Ptr create_preset_cache_task()
  {
    auto cache = std::make_shared<ScheduleCache>();

    // Gain Write access
    ScheduleCache::TaskRestricted task_write_token;
    auto task_handler = cache->make_task_handler(task_write_token);

    auto task = std::make_shared<data::Task>(
      "task_1",
      "test_isolated_task",
      data::Time(24 * 60 * 60 * pow(10, 9)),   // 1970-01-01 00:00:00 (UTC)
      "pending"
    );
    task->task_details = R"({
      "information": "some_information"
    })"_json;

    // Create isolated event
    task_handler->emplace(task);

    return cache;
  }

  static rmf2_scheduler::cache::ScheduleCache::Ptr create_preset_cache_process()
  {
    auto cache = std::make_shared<ScheduleCache>();

    // Gain Write access
    ScheduleCache::TaskRestricted task_write_token;
    auto task_handler = cache->make_task_handler(task_write_token);

    ScheduleCache::ProcessRestricted process_write_token;
    auto process_handler = cache->make_process_handler(process_write_token);


    ScheduleCache::SeriesRestricted series_write_token;
    auto series_handler = cache->make_series_handler(series_write_token);

    // Create process and corresponding events
    // Add events first
    std::vector<data::Task::Ptr> process_tasks(5);
    std::string process_task_prefix = "process_task_";
    std::string process_task_type = "test_process_task";
    std::string process_id = "process_1";
    for (size_t i = 0; i < process_tasks.size(); i++) {
      // Create event_id
      std::ostringstream oss;
      oss << process_task_prefix << i + 1;
      std::string process_task_id = oss.str();

      // Create event
      process_tasks[i] = std::make_shared<data::Task>(
        process_task_id,
        process_task_type,
        data::Time(24 * 60 * 60 * pow(10, 9)),
        "pending"
      );
      process_tasks[i]->duration = std::chrono::minutes(5);

      // Update process ID
      process_tasks[i]->process_id = process_id;

      // Add event to cache
      task_handler->emplace(process_tasks[i]);
    }

    // Create process
    data::Process::Ptr process = std::make_shared<data::Process>();
    process->id = process_id;
    data::Graph & graph = process->graph;
    for (size_t i = 0; i < process_tasks.size(); i++) {
      graph.add_node(process_tasks[i]->id);
    }
    graph.add_edge(process_tasks[0]->id, process_tasks[2]->id, data::Edge("hard"));
    graph.add_edge(process_tasks[1]->id, process_tasks[2]->id, data::Edge("soft"));
    graph.add_edge(process_tasks[2]->id, process_tasks[3]->id, data::Edge("hard"));
    graph.add_edge(process_tasks[2]->id, process_tasks[4]->id, data::Edge("soft"));

    // Add process to cache
    process_handler->emplace(process);

    return cache;
  }

  static rmf2_scheduler::cache::ScheduleCache::Ptr create_preset_cache_series_task()
  {
    auto cache = create_preset_cache_task();

    // Gain Write access
    ScheduleCache::SeriesRestricted series_write_token;
    auto series_handler = cache->make_series_handler(series_write_token);

    const std::string cron_string = "0 0 0 * * ?";

    // create series
    const std::string series_id = "series_1";
    auto series_to_add = std::make_shared<data::Series>(
      series_id,
      "task",
      data::Occurrence(data::Time(24 * 60 * 60 * pow(10, 9)), "task_1"),
      cron_string,
      "UTC"
    );
    series_handler->emplace(series_to_add);

    return cache;
  }

  static rmf2_scheduler::cache::ScheduleCache::Ptr create_preset_cache_series_process()
  {
    auto cache = create_preset_cache_process();

    // Gain Write access
    ScheduleCache::SeriesRestricted series_write_token;
    auto series_handler = cache->make_series_handler(series_write_token);

    const std::string cron_string = "0 0 0 * * ?";

    // create series
    const std::string series_id = "series_1";
    auto series_to_add = std::make_shared<data::Series>(
      series_id,
      "process",
      data::Occurrence(data::Time(24 * 60 * 60 * pow(10, 9)), "process_1"),
      cron_string,
      "UTC"
    );

    series_handler->emplace(series_to_add);

    return cache;
  }


  static rmf2_scheduler::cache::TaskHandler::Ptr create_task_handler(ScheduleCache::Ptr cache)
  {
    // Gain Write access
    ScheduleCache::TaskRestricted task_write_token;
    return cache->make_task_handler(task_write_token);
  }

  static rmf2_scheduler::cache::ProcessHandler::Ptr create_process_handler(ScheduleCache::Ptr cache)
  {
    ScheduleCache::ProcessRestricted process_write_token;
    return cache->make_process_handler(process_write_token);
  }

  static rmf2_scheduler::cache::SeriesHandler::Ptr create_series_handler(ScheduleCache::Ptr cache)
  {
    ScheduleCache::SeriesRestricted series_write_token;
    return cache->make_series_handler(series_write_token);
  }
};

}  // namespace cache
}  // namespace rmf2_scheduler


// test series add
// test series expand until
// test series update cron
// test series update until
// test series update occurrence time
// test series delete occurrence
// test series delete all

class TestCacheSeriesAction : public ::testing::Test
{
};

TEST_F(TestCacheSeriesAction, invalid_action) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

  // Create invalid action
  SeriesAction series_action(
    action_type::EVENT_ADD,
    ActionPayload()
  );

  // Validate
  std::string error;
  EXPECT_FALSE(series_action.validate(schedule_cache, error));
  EXPECT_EQ(error, "SERIES action invalid [EVENT_ADD].");

  // Apply
  EXPECT_THROW_EQ(
    series_action.apply(),
    std::runtime_error("Action invalid!"));
}

TEST_F(TestCacheSeriesAction, undefined_action) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

  // Create invalid process action
  SeriesAction series_action(
    "SERIES_INVALID_ACTION",
    ActionPayload()
  );

  // Validate
  std::string error;
  EXPECT_FALSE(series_action.validate(schedule_cache, error));
  EXPECT_EQ(error, "Undefined SERIES action [SERIES_INVALID_ACTION].");

  // Apply
  EXPECT_THROW_EQ(
    series_action.apply(),
    std::runtime_error("Action invalid!"));
}

TEST_F(TestCacheSeriesAction, series_add) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_series_task();

    // Create process add action
    SeriesAction series_action(
      action_type::SERIES_ADD,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "SERIES_ADD failed, no series found in payload.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }

  // series id already exits
  // more than one occurrence
  // no input occurrence for reference
  // wrong series type
  // event occurrence does not exist
  // event occurrence part of a process
  // event time does not match cron
  // process does not exist
  // process time does not match cron

  {  // Invalid series ID id already exists
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_series_task();
    auto task_to_add = std::make_shared<Task>(
      "task_1",
      "test_isolated_task",
      Time(24 * 60 * 60 * pow(10, 9)),   // 1970-01-01 00:00:00 (UTC)
      "pending"
    );

    // fire at midnight
    const std::string cron_string = "0 0 0 * * ?";
    auto series_to_add = std::make_shared<Series>(
      "series_1",
      "task",
      Occurrence(task_to_add->start_time, "task_1"),
      cron_string,
      "SGT"
    );

    SeriesAction series_action(
      action_type::SERIES_ADD,
      ActionPayload().series(series_to_add)
    );

    // Validate
    std::string error;
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "SERIES_ADD failed, series [series_1] already exists.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }

  { // reject when series occurence type is specified wrongly
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_task();
    auto task_handler = TestScheduleCache::create_task_handler(schedule_cache);

    const std::string cron_string = "0 0 0 * * ?";

    // create series
    const std::string series_id = "series_1";
    auto series_to_add = std::make_shared<Series>(
      series_id,
      "process",
      Occurrence(Time(24 * 60 * 60 * pow(10, 9)), "task_1"),
      cron_string,
      "UTC"
    );

    SeriesAction series_action(
      action_type::SERIES_ADD,
      ActionPayload().series(series_to_add)
    );

    std::string error;
    std::string expected_error =
      "SERIES_ADD failed, series [series_1] initial occurrence, Process [task_1] does not exist.";
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(error, expected_error);

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }

  {  // reject when task is part of a process
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_process();

    const std::string cron_string = "0 0 0 * * ?";

    const std::string process_task_id = "process_task_1";

    // create series
    const std::string series_id = "series_1";
    auto series_to_add = std::make_shared<Series>(
      series_id,
      "task",
      Occurrence(Time(24 * 60 * 60 * pow(10, 9)), process_task_id),
      cron_string,
      "UTC"
    );

    SeriesAction series_action(
      action_type::SERIES_ADD,
      ActionPayload().series(series_to_add)
    );

    std::string error;
    std::string expected_error =
      "SERIES_ADD failed, occurrence task [" + process_task_id +
      "] is part of Process [process_1].";
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(error, expected_error);

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }

  {  // test reject when more than one occurrence when added
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();
    auto task_handler = TestScheduleCache::create_task_handler(schedule_cache);

    // fire everyday at midnight
    const std::string cron_string = "0 0 0 * * ?";

    // add two tasks
    auto task1_to_add = std::make_shared<Task>(
      "task_1",
      "test_task",
      Time(24 * 60 * 60 * pow(10, 9)),   // 1970-02-01 00:00:00 (UTC)
      "pending"
    );

    auto task2_to_add = std::make_shared<Task>(
      "task_2",
      "test_task",
      Time(24 * 60 * 60 * pow(10, 9) * 2),  // 1970-03-01 00:00:00 (UTC)
      "pending"
    );

    // add task to cache
    task_handler->emplace(task1_to_add);
    task_handler->emplace(task2_to_add);
    std::vector<Occurrence> task_occurrences = {
      Occurrence(task1_to_add->start_time, "task_1"),
      Occurrence(task2_to_add->start_time, "task_2")};

    auto series_to_add = std::make_shared<Series>(
      "series_1",
      "task",
      task_occurrences,
      cron_string,
      "UTC"
    );

    SeriesAction series_action(
      action_type::SERIES_ADD,
      ActionPayload().series(series_to_add)
    );

    // Validate
    std::string error;
    std::string expected_error = "SERIES_ADD failed, series [" +
      series_to_add->id() + "] there should only be one occurrence provided.";
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      expected_error
    );

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }

  {  // Test add series for single task
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_task();

    // fire everyday at midnight
    const std::string cron_string = "0 0 0 * * ?";

    // create series
    const std::string series_id = "series_1";
    auto series_to_add = std::make_shared<Series>(
      series_id,
      "task",
      Occurrence(Time(24 * 60 * 60 * pow(10, 9)), "task_1"),
      cron_string,
      "UTC"
    );

    SeriesAction series_action(
      action_type::SERIES_ADD,
      ActionPayload().series(series_to_add)
    );

    // Validate, should always be true
    std::string error;
    ASSERT_TRUE(series_action.validate(schedule_cache, error));
    ASSERT_TRUE(error.empty());

    // Check that the the subsequent tasks in the series occurrences are replicated correctly
    series_action.apply();
    auto series_added = schedule_cache->get_series(series_id);
    auto occurrences = series_added->occurrences();
    ASSERT_EQ(occurrences.size(), 31);
    for (const auto & occ : occurrences) {
      auto task = schedule_cache->get_task(occ.id);

      // Check that task details are copied correctly
      ASSERT_TRUE(task->task_details.contains("information"));
      ASSERT_EQ(task->task_details["information"].get<std::string>(), "some_information");
    }
  }

  {  // Test add series for single process
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_process();

    // fire everyday at midnight
    const std::string cron_string = "0 0 0 * * ?";

    // create series
    const std::string series_id = "series_1";
    auto series_to_add = std::make_shared<Series>(
      series_id,
      "process",
      Occurrence(Time(24 * 60 * 60 * pow(10, 9)), "process_1"),
      cron_string,
      "UTC"
    );

    SeriesAction series_action(
      action_type::SERIES_ADD,
      ActionPayload().series(series_to_add)
    );

    // Validate, should always be true
    std::string error;
    ASSERT_TRUE(series_action.validate(schedule_cache, error));
    ASSERT_TRUE(error.empty());

    series_action.apply();
    auto series = schedule_cache->get_series(series_id);
    const auto occurrences = series->occurrences();
    const auto reference_process = schedule_cache->get_process("process_1");
    ASSERT_EQ(occurrences.size(), 31);
    for (const auto & occ : occurrences) {
      // check that process is replicated correctly
      const auto process = schedule_cache->get_process(occ.id);
      ASSERT_EQ(process->graph.size(), reference_process->graph.size());

      // check that the tasks within the process are replicated correctly
      // TODO(kai): Use process validation when the PR is merged in.
      // Currently there is no way to determine that two graphs are isomorphic.
    }
  }
}


TEST_F(TestCacheSeriesAction, series_expand_until) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload no ID
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create empty process attach node action
    SeriesAction series_action(
      action_type::SERIES_EXPAND_UNTIL,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "SERIES_EXPAND_UNTIL failed, id and until must be set in payload.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }

  {  // series does not exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    SeriesAction series_action(
      action_type::SERIES_EXPAND_UNTIL,
      ActionPayload()
      .id("no_such_series")
      .until(
        Time(
          24 * 60 * 60 * pow(10, 9) * 365    // one year after epoch time
      ))
    );
    // Validate
    std::string error;
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "SERIES_EXPAND_UNTIL failed, series [no_such_series] does not exist.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }

  {  // expand until is before last occurrence.
    // Create preset cache
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_task();

    // add series action to expand
    auto series_to_add = std::make_shared<Series>(
      "series_1",
      "task",
      Occurrence(Time(24 * 60 * 60 * pow(10, 9)), "task_1"),
      "0 0 0 * * ?",
      "SGT"
    );

    SeriesAction series_action_add(
      action_type::SERIES_ADD,
      ActionPayload().series(series_to_add)
    );
    std::string error;
    EXPECT_TRUE(series_action_add.validate(schedule_cache, error));
    series_action_add.apply();

    // Create process attach node action
    SeriesAction series_action(
      action_type::SERIES_EXPAND_UNTIL,
      ActionPayload()
      .id("series_1")
      .until(
        Time(
          24 * 60 * 60 * pow(10, 9) * 10    // 10 days
      ))
    );
    // Validate
    error = "";
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(error, "SERIES_EXPAND_UNTIL failed, provided time is before the last occurrence.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }

  {  // expand until is after last occurrence
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();
    auto task = std::make_shared<Task>(
      "task_1",
      "test_isolated_task",
      Time(24 * 60 * 60 * pow(10, 9)),  // 1970-02-01 00:00:00 (UTC)
      "pending"
    );

    // Add task
    auto task_handler = TestScheduleCache::create_task_handler(schedule_cache);
    task_handler->emplace(task);

    const std::string cron_string = "0 0 0 * * ?";
    const std::string series_id = "series_1";
    auto series_to_add = std::make_shared<Series>(
      series_id,
      "task",
      Occurrence(task->start_time, task->id),
      cron_string,
      "UTC"
    );

    SeriesAction series_action_add(
      action_type::SERIES_ADD,
      ActionPayload().series(series_to_add)
    );

    // Add series
    std::string error;
    ASSERT_TRUE(series_action_add.validate(schedule_cache, error));
    series_action_add.apply();

    // Expand until one day before first occurrence
    SeriesAction series_action_expand(
      action_type::SERIES_EXPAND_UNTIL,
      ActionPayload().id("series_1").until(Time(0))
    );

    error = "";
    ASSERT_FALSE(series_action_expand.validate(schedule_cache, error));
    ASSERT_EQ(error, "SERIES_EXPAND_UNTIL failed, provided time is before the frist occurrence.");
  }

  {  // Test validate expansion series task
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_series_task();
    SeriesAction series_action(
      action_type::SERIES_EXPAND_UNTIL,
      ActionPayload()
      .id("series_1")
      .until(
        Time(
          24 * 60 * 60 * pow(10, 9) * 60    // 60 days
      ))
    );

    std::string error;
    ASSERT_TRUE(series_action.validate(schedule_cache, error));
    ASSERT_EQ(error, "");
    series_action.apply();

    // Check if it was expanded the correct number of times
    auto expanded_series = schedule_cache->get_series("series_1");
    const auto occurrences = expanded_series->occurrences();
    ASSERT_EQ(occurrences.size(), 60);

    // Check if the time is set correctly
    for (int i = 0; i < occurrences.size(); i++) {
      const auto task = schedule_cache->get_task(occurrences[i].id);
      ASSERT_EQ(task->start_time, Time(24 * 60 * 60 * pow(10, 9) * (i + 1)));
    }
  }

  { // Test validate expansion series process
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_series_process();
    SeriesAction series_action(
      action_type::SERIES_EXPAND_UNTIL,
      ActionPayload()
      .id("series_1")
      .until(
        Time(
          24 * 60 * 60 * pow(10, 9) * 60    // 60 days
      ))
    );

    std::string error;
    ASSERT_TRUE(series_action.validate(schedule_cache, error));
    ASSERT_EQ(error, "");
    series_action.apply();

    // Check if it was expanded the correct number of times
    auto expanded_series = schedule_cache->get_series("series_1");
    const auto occurrences = expanded_series->occurrences();
    ASSERT_EQ(occurrences.size(), 60);

    // Check if the time is set correctly
    for (int i = 0; i < occurrences.size(); i++) {
      const auto process = schedule_cache->get_process(occurrences[i].id);
      auto nodes = process->graph.get_all_nodes();
      for (const auto n : nodes) {
        const auto task = schedule_cache->get_task(n.first);
        ASSERT_EQ(task->start_time, 24 * 60 * 60 * pow(10, 9) * (i + 1));
      }
    }
  }
}


TEST_F(TestCacheSeriesAction, series_update) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // validation should fail as it is currently not supported
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_series_task();

    auto series_to_update = std::make_shared<Series>(
      "series_1",
      "task",
      Occurrence(Time(24 * 60 * 60 * pow(10, 9)), "task_1"),
      "0 0 0 * * ?",
      "UTC"
    );

    SeriesAction series_action(
      action_type::SERIES_UPDATE,
      ActionPayload().series(series_to_update)
    );

    std::string error;
    ASSERT_FALSE(series_action.validate(schedule_cache, error));
    ASSERT_EQ(error, "SERIES_UPDATE currently not supported.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }
}

TEST_F(TestCacheSeriesAction, series_update_cron) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // validation should fail as it is currently not supported
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_series_task();

    std::string new_cron_string = "0 0 1 * * ?";  // fire everyday at 1am
    SeriesAction series_action(
      action_type::SERIES_UPDATE_CRON,
      ActionPayload().id("series_1").cron(new_cron_string)
    );

    std::string error;
    ASSERT_FALSE(series_action.validate(schedule_cache, error));
    ASSERT_EQ(error, "SERIES_UPDATE_CRON is currently not supported.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }
}

TEST_F(TestCacheSeriesAction, series_update_until) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // validation should fail as it is currently not supported
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_series_task();

    SeriesAction series_action(
      action_type::SERIES_UPDATE_UNTIL,
      ActionPayload().id("series_1").until(Time(24 * 60 * 60 * pow(10, 9) * 365))
    );

    std::string error;
    ASSERT_FALSE(series_action.validate(schedule_cache, error));
    ASSERT_EQ(error, "SERIES_UPDATE_UNTIL currently not supported.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }
}

TEST_F(TestCacheSeriesAction, series_update_occurrence) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // validation should fail as it is currently not supported
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_series_task();

    SeriesAction series_action(
      action_type::SERIES_UPDATE_OCCURRENCE,
      ActionPayload()
      .id("series_1")
      .task(
        std::make_shared<Task>(
          "task_1",
          "test_isolated_task",
          Time(24 * 60 * 60 * pow(10, 9)),
          "pending"
      ))
    );

    std::string error;
    ASSERT_FALSE(series_action.validate(schedule_cache, error));
    ASSERT_EQ(error, "SERIES_UPDATE_OCCURRENCE currently not supported.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }
}

TEST_F(TestCacheSeriesAction, series_update_occurrence_time) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // Invalid payload
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();

    // Create empty process attach node action
    SeriesAction series_action(
      action_type::SERIES_UPDATE_OCCURRENCE_TIME,
      ActionPayload()
    );

    // Validate
    std::string error;
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "SERIES_UPDATE_OCCURRENCE_TIME failed, id, occurrence_id and time must be set in payload.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }

  {  // series does not exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_series_task();

    SeriesAction series_action(
      action_type::SERIES_UPDATE_OCCURRENCE_TIME,
      ActionPayload()
      .id("no_such_series")
      .occurrence_id("task_1")
      .occurrence_time(
        Time(
          3 * 60 * 60 * pow(10, 9)    // shift task 3 hours ahead
      ))
    );
    // Validate
    std::string error;
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "SERIES_UPDATE_OCCURRENCE_TIME failed, series [no_such_series] does not exist.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }


  {  // occurrence does not exist
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_preset_cache_series_task();

    SeriesAction series_action(
      action_type::SERIES_UPDATE_OCCURRENCE_TIME,
      ActionPayload()
      .id("series_1")
      .occurrence_id("no_such_occurrence")
      .occurrence_time(
        Time(
          3 * 60 * 60 * pow(10, 9)    // shift task 3 hours ahead
      ))
    );
    // Validate
    std::string error;
    EXPECT_FALSE(series_action.validate(schedule_cache, error));
    EXPECT_EQ(
      error,
      "SERIES_UPDATE_OCCURRENCE_TIME failed, occurrence [no_such_occurrence] does not exist.");

    // Apply
    EXPECT_THROW_EQ(
      series_action.apply(),
      std::runtime_error("Action invalid!"));
  }

  {  //  reject when occurrence is shifted to before/after the previous/next occurrence
    // Create series with exactly two occurrences
    ScheduleCache::Ptr schedule_cache = TestScheduleCache::create_empty_cache();
    auto task_handler = TestScheduleCache::create_task_handler(schedule_cache);

    // fire everyday at midnight
    const std::string cron_string = "0 0 0 * * ?";

    // add two tasks
    auto task1_to_add = std::make_shared<Task>(
      "task_1",
      "test_task",
      Time(24 * 60 * 60 * pow(10, 9)),   // 1970-02-01 00:00:00 (UTC)
      "pending"
    );

    auto task2_to_add = std::make_shared<Task>(
      "task_2",
      "test_task",
      Time(24 * 60 * 60 * pow(10, 9) * 2),  // 1970-03-01 00:00:00 (UTC)
      "pending"
    );

    // add task to cache
    task_handler->emplace(task1_to_add);
    task_handler->emplace(task2_to_add);
    std::vector<Occurrence> task_occurrences = {
      Occurrence(task1_to_add->start_time, "task_1"),
      Occurrence(task2_to_add->start_time, "task_2")};

    auto series_to_add = std::make_shared<Series>(
      "series_1",
      "task",
      task_occurrences,
      cron_string,
      "UTC"
    );

    auto series_handler = TestScheduleCache::create_series_handler(schedule_cache);
    series_handler->emplace(series_to_add);

    {  // updated time exactly a previous occurrence's time
      SeriesAction series_action(
        action_type::SERIES_UPDATE_OCCURRENCE_TIME,
        ActionPayload()
        .id("series_1")
        .occurrence_id("task_2")
        .occurrence_time(
          Time(
            24 * 60 * 60 * pow(10, 9)    // eaxctly at first occurrence
        ))
      );

      // Validate
      std::string error;
      std::string expected_error =
        "SERIES_UPDATE_OCCURRENCE_TIME failed, new update time [Jan 02 00:00:00 1970] is"
        " before or equivalent to previous occurrence time at [Jan 02 00:00:00 1970].";
      EXPECT_FALSE(series_action.validate(schedule_cache, error));
      EXPECT_EQ(
        error,
        expected_error);

      // Apply
      EXPECT_THROW_EQ(
        series_action.apply(),
        std::runtime_error("Action invalid!"));
    }
    {  // update time before previous occurrence's time
      SeriesAction series_action(
        action_type::SERIES_UPDATE_OCCURRENCE_TIME,
        ActionPayload()
        .id("series_1")
        .occurrence_id("task_2")
        .occurrence_time(
          Time(
            0    // eaxctly at first occurrence
        ))
      );

      // Validate
      std::string error;
      std::string expected_error =
        "SERIES_UPDATE_OCCURRENCE_TIME failed, new update time [Jan 01 00:00:00 1970] is"
        " before or equivalent to previous occurrence time at [Jan 02 00:00:00 1970].";
      EXPECT_FALSE(series_action.validate(schedule_cache, error));
      EXPECT_EQ(
        error,
        expected_error);

      // Apply
      EXPECT_THROW_EQ(
        series_action.apply(),
        std::runtime_error("Action invalid!"));
    }

    {  // updated time exactly at next occurrence's time
      SeriesAction series_action(
        action_type::SERIES_UPDATE_OCCURRENCE_TIME,
        ActionPayload()
        .id("series_1")
        .occurrence_id("task_1")
        .occurrence_time(
          Time(
            24 * 60 * 60 * pow(10, 9) * 2    // exactly at next occurrence
        ))
      );

      // Validate
      std::string error;
      std::string expected_error =
        "SERIES_UPDATE_OCCURRENCE_TIME failed, new update time [Jan 03 00:00:00 1970] is"
        " after or equivalent to next occurrence time at [Jan 03 00:00:00 1970].";
      EXPECT_FALSE(series_action.validate(schedule_cache, error));
      EXPECT_EQ(
        error,
        expected_error);

      // Apply
      EXPECT_THROW_EQ(
        series_action.apply(),
        std::runtime_error("Action invalid!"));
    }
    {  // updated time after next occurrence's time
      SeriesAction series_action(
        action_type::SERIES_UPDATE_OCCURRENCE_TIME,
        ActionPayload()
        .id("series_1")
        .occurrence_id("task_1")
        .occurrence_time(
          Time(
            24 * 60 * 60 * pow(10, 9) * 3    // after next occurrence
        ))
      );

      // Validate
      std::string error;
      std::string expected_error =
        "SERIES_UPDATE_OCCURRENCE_TIME failed, new update time [Jan 04 00:00:00 1970] is"
        " after or equivalent to next occurrence time at [Jan 03 00:00:00 1970].";
      EXPECT_FALSE(series_action.validate(schedule_cache, error));
      EXPECT_EQ(
        error,
        expected_error);

      // Apply
      EXPECT_THROW_EQ(
        series_action.apply(),
        std::runtime_error("Action invalid!"));
    }
  }
}
