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

#include <sstream>

#include "gtest/gtest.h"
#include "rmf_scheduler/scheduler.hpp"
#include "rmf_scheduler/test_utils.hpp"
#include "nlohmann/json.hpp"

// Test Add Schedule
class TestAddSchedule : public ::testing::Test
{
protected:
  void SetUp() override
  {
    using namespace rmf_scheduler;  // NOLINT(build/namespaces)
    // Don't expand series
    scheduler_ = std::make_unique<Scheduler>(
      SchedulerOptions()
      .expand_series_automatically(false)  // Don't expand series
      .allow_past_events_duration(2e8));
  }

  rmf_scheduler::ErrorCode load_valid_schedule()
  {
    using namespace rmf_scheduler;  // NOLINT(build/namespaces)
    auto valid_str =
      test_utils::init_json_from_file("json/add_valid.json");
    auto error_code = scheduler_->handle_add_schedule(nlohmann::json::parse(valid_str));
    current_schedule_json_str_ = scheduler_->handle_get_schedule(R"({})"_json).dump(2);
    return error_code;
  }

  bool schedule_unchanged()
  {
    bool result = scheduler_->handle_get_schedule(R"({})"_json).dump(2) ==
      current_schedule_json_str_;
    std::cout << (result ? "Schedule unchanged" : "Schedule changed") << std::endl;
    return result;
  }

  void TearDown() override
  {
  }

  std::unique_ptr<rmf_scheduler::Scheduler> scheduler_;
  std::string current_schedule_json_str_;
};

TEST_F(TestAddSchedule, AddValidSchedule)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Add Valid JSON
  auto error_code = load_valid_schedule();
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(error_code.val, ErrorCode::SUCCESS);

  std::cout << current_schedule_json_str_ << std::endl;
}

// Test Exception Handling
// Invalid schema
TEST_F(TestAddSchedule, InvalidSchema)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Add Valid JSON
  load_valid_schedule();

  // Invalid JSON
  auto error_code = scheduler_->handle_add_schedule("random_stuff");
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_SCHEMA
  );

  // No changes to schedule
  EXPECT_TRUE(schedule_unchanged());

  // Minimal viable empty schedule
  error_code = scheduler_->handle_add_schedule(R"({"events":{}})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(error_code.val, ErrorCode::SUCCESS);

  // No changes to schedule
  EXPECT_TRUE(schedule_unchanged());
}

// Test Invalid ID
TEST_F(TestAddSchedule, InvalidID)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Add Valid JSON
  load_valid_schedule();

  // Invalid Event ID
  // This time should spit out Event ID invalid error code
  auto valid_str =
    test_utils::init_json_from_file("json/add_valid.json");
  auto error_code = scheduler_->handle_add_schedule(nlohmann::json::parse(valid_str));
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_ID |
    ErrorCode::INVALID_EVENT);
  // Internal schedule should not get changed
  EXPECT_TRUE(schedule_unchanged());

  // This should spit out Dependency ID invalid error code
  error_code = scheduler_->handle_add_schedule(
    R"({"events":{}, "dependencies": {"dag-1":{}}})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_ID |
    ErrorCode::INVALID_DEPENDENCY);
  // Internal schedule should not get changed
  EXPECT_TRUE(schedule_unchanged());

  // This should spit out Series ID invalid error code
  error_code = scheduler_->handle_add_schedule(
    R"({"events":{}, "series":
      {"series-1":{"cron": "",
      "timezone":"UTC","occurrences":[]}}})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_ID |
    ErrorCode::INVALID_SERIES);
  // Internal schedule should not get changed
  EXPECT_TRUE(schedule_unchanged());
}

// Test Invalid dependency logic
TEST_F(TestAddSchedule, InvalidDependencyLogic)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Add Valid JSON
  load_valid_schedule();

  // Invalid dependency graph cyclic
  auto cyclic_dag_schedule_json_str =
    test_utils::init_json_from_file("json/add_cyclic_dag.json");
  auto error_code = scheduler_->handle_add_schedule(
    nlohmann::json::parse(cyclic_dag_schedule_json_str));
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_LOGIC |
    ErrorCode::INVALID_DEPENDENCY);
  // Internal schedule should not get changed
  EXPECT_TRUE(schedule_unchanged());
}

// Test Invalid dependency graph
TEST_F(TestAddSchedule, InvalidSeriesLogic)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Add Valid JSON
  load_valid_schedule();

  // Invalid Series Logic (Empty series)
  auto series_empty_schedule_json_str =
    test_utils::init_json_from_file("json/add_series_empty.json");

  auto error_code = scheduler_->handle_add_schedule(
    nlohmann::json::parse(series_empty_schedule_json_str));
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_LOGIC |
    ErrorCode::INVALID_SERIES
  );
  // Internal schedule should not get changed
  EXPECT_TRUE(schedule_unchanged());

  // Invalid Series Logic (Invalid cron)
  auto invalid_cron_schedule_json_str =
    test_utils::init_json_from_file("json/add_invalid_cron.json");

  error_code = scheduler_->handle_add_schedule(
    nlohmann::json::parse(invalid_cron_schedule_json_str));
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_LOGIC |
    ErrorCode::INVALID_SERIES
  );
  // Internal schedule should not get changed
  EXPECT_TRUE(schedule_unchanged());
}

/////////////////////////////////////////

// Test Update Schedule
class TestUpdateSchedule : public TestAddSchedule
{
};

TEST_F(TestUpdateSchedule, UpdateValidSchedule)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Add Valid JSON
  load_valid_schedule();

  auto valid_json =
    nlohmann::json::parse(test_utils::init_json_from_file("json/update_valid.json"));
  auto error_code = scheduler_->handle_update_schedule(valid_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(error_code.val, ErrorCode::SUCCESS);
  auto new_schedule_json = scheduler_->handle_get_schedule(R"({})"_json);
  // std::cout << new_schedule_json.dump(2) << std::endl;

  // Verify that the updated schedule is updated correctly
  const auto & handler = scheduler_->schedule_handler_const();
  // Verify event start time is updated
  const data::Event & clean_task1 = handler.get_event("clean_task1");
  uint64_t new_clean_task1_start_time =
    static_cast<uint64_t>(
    valid_json["events"]["clean_task1"]["start_time"].get<double>() * 1e9);
  uint64_t new_clean_task1_duration =
    static_cast<uint64_t>(
    valid_json["events"]["clean_task1"]["duration"].get<double>() * 1e9);
  EXPECT_EQ(
    clean_task1.start_time,
    new_clean_task1_start_time
  );

  // Verify DAG is updated
  auto deps = handler.get_dag("dag-1").get_dependency_info("go_to_place_task1");
  EXPECT_EQ(deps[0], "clean_task1");

  // Verify event start time within DAG is updated
  const data::Event & go_to_place_task1 = handler.get_event("go_to_place_task1");
  EXPECT_EQ(
    go_to_place_task1.start_time,
    new_clean_task1_start_time + new_clean_task1_duration
  );

  // Verify series is updated
  std::string cron_str = handler.get_series("series-1").cron();
  EXPECT_EQ(cron_str, "08 51 21 ? * Sat,Sun");
}

TEST_F(TestUpdateSchedule, InvalidID)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Add Valid JSON
  load_valid_schedule();

  // Invalid Event ID
  // This should spit out Event ID invalid error code
  auto invalid_event_id_json =
    nlohmann::json::parse(
    test_utils::init_json_from_file("json/update_invalid_event_id.json"));
  auto error_code = scheduler_->handle_update_schedule(
    R"({"events":{"abcdefg": {
      "description": "Invalid event id",
      "type": "default/robot_task",
      "start_time": 1680961868,
      "event_details": {}
    }}})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_ID |
    ErrorCode::INVALID_EVENT);
  // Internal schedule should not get changed
  EXPECT_TRUE(schedule_unchanged());

  // Invalid Dependency ID
  // This should spit out Dependency ID invalid error code
  error_code = scheduler_->handle_update_schedule(
    R"({"events":{}, "dependencies": {"abcdefgg":{}}})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_ID |
    ErrorCode::INVALID_DEPENDENCY);
  // Internal schedule should not get changed
  EXPECT_TRUE(schedule_unchanged());

  // This should spit out Series ID invalid error code
  error_code = scheduler_->handle_update_schedule(
    R"({"events":{}, "series":
      {"series-2":{"cron": "",
      "timezone":"UTC","occurrences":[]}}})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_ID |
    ErrorCode::INVALID_SERIES);
  // Internal schedule should not get changed
  EXPECT_TRUE(schedule_unchanged());

  // Cannot change series from a dag series to an event series
  error_code = scheduler_->handle_update_schedule(
    R"({"events": {}, "series":{
    "series-1": {
      "cron": "48 42 21 ? * Sat,Sun",
      "timezone": "Asia/Singapore",
      "id_prefix": "series-",
      "occurrences": ["clean_task1"]
    }
    }})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_ID |
    ErrorCode::INVALID_SERIES);
  // Internal schedule should not get changed
  EXPECT_TRUE(schedule_unchanged());
}

TEST_F(TestUpdateSchedule, InvalidDependencyLogic)
{
  // TODO(Briancbn): Add tests
}

TEST_F(TestUpdateSchedule, InvalidSeriesLogic)
{
  // TODO(Briancbn): Add tests
}


/////////////////////////////////////////

// Test Delete Schedule
class TestDeleteSchedule : public TestAddSchedule
{
};

// Test Delete Schedule
TEST_F(TestDeleteSchedule, DeleteValidDAGSeries)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Add Valid JSON
  load_valid_schedule();

  // Delete a single event
  // This causes DAG to change resulting in series making an exception for the old dag
  auto error_code = scheduler_->handle_delete_schedule(R"({ "event_ids": ["clean_task1"]})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(error_code.val, ErrorCode::SUCCESS);

  const auto & handler = scheduler_->schedule_handler_const();
  // Verify event is deleted
  EXPECT_FALSE(handler.events_handler_const().has_event("clean_task1"));
  const data::Series & series1 = handler.get_series("series-1");
  std::string last_occurrence_id = series1.get_last_occurrence().id;
  // Verify series is expanded
  EXPECT_NE(last_occurrence_id, "dag-1");
  // Verify dags are automatically added
  EXPECT_TRUE(handler.dags_const().find(last_occurrence_id) != handler.dags_const().end());

  // Print out schedule for debugging
  auto new_schedule_json = scheduler_->handle_get_schedule(R"({})"_json);
  // std::cout << new_schedule_json.dump(2) << std::endl;


  // Delete a single event, resulting in dag being empty
  // This causes DAG and the occurrence within the series to be deleted
  error_code = scheduler_->handle_delete_schedule(
    R"({ "event_ids": ["go_to_place_task1"], "dependency_ids": ["dag-1"]})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(error_code.val, ErrorCode::SUCCESS);

  // Verify DAG is deleted
  EXPECT_FALSE(handler.dags_const().find("dag-1") != handler.dags_const().end());

  // Verify deleted DAG no longer exist in the series
  auto all_occurrences = handler.get_series("series-1").occurrences();
  bool series1_deleted = true;
  for (auto & occurrence : all_occurrences) {
    if (occurrence.id == "series-1") {
      series1_deleted = false;
    }
  }
  EXPECT_TRUE(series1_deleted);

  new_schedule_json = scheduler_->handle_get_schedule(R"({})"_json);
  // std::cout << new_schedule_json.dump(2) << std::endl;


  // Delete a dag event series, resulting in all dags && event deleted
  error_code = scheduler_->handle_delete_schedule(R"({ "series_ids": ["series-1"]})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(error_code.val, ErrorCode::SUCCESS);

  // Verify the schedule is completely empty
  EXPECT_TRUE(handler.events_handler_const().get_all_events().empty());
  EXPECT_TRUE(handler.dags_const().empty());
  EXPECT_TRUE(handler.series_map_const().empty());
  new_schedule_json = scheduler_->handle_get_schedule(R"({})"_json);
  // std::cout << new_schedule_json.dump(2) << std::endl;
}
