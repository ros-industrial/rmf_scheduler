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

#include <fstream>
#include <sstream>

#include "gtest/gtest.h"
#include "rmf_scheduler/scheduler.hpp"
#include "nlohmann/json.hpp"

namespace rmf_scheduler::test_utils
{

std::string init_json_from_file(
  const std::string & path,
  bool relative = true)
{
  std::ifstream f_invalid_cron(
    relative ? std::string(TEST_DIRECTORY) + path : path);
  std::stringstream b_invalid_cron;
  b_invalid_cron << f_invalid_cron.rdbuf();
  return b_invalid_cron.str();
}

std::string to_pretty_json(
  const std::string & in,
  int indent = 4)
{
  nlohmann::json json = nlohmann::json::parse(in);
  return json.dump(indent);
}

}  // namespace rmf_scheduler::test_utils

TEST(TestScheduler, AddScheduleJSONException)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Invalid JSON
  std::unique_ptr<Scheduler> scheduler = std::make_unique<Scheduler>();
  auto error_code = scheduler->add_schedule("random_stuff");
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_SCHEMA
  );

  // Minimal viable empty schedule
  error_code = scheduler->add_schedule(R"({"events":{}})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(error_code.val, ErrorCode::SUCCESS);

  // Invalid Event ID
  auto valid_str =
    test_utils::init_json_from_file("json/valid.json");
  // This time should be valid
  error_code = scheduler->add_schedule(nlohmann::json::parse(valid_str));
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(error_code.val, ErrorCode::SUCCESS);

  auto all_schedule_str = scheduler->get_schedule(R"({})"_json);
  std::cout << test_utils::to_pretty_json(all_schedule_str) << std::endl;

  // This time should spit out Event ID invalid error code
  error_code = scheduler->add_schedule(nlohmann::json::parse(valid_str));
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_ID |
    ErrorCode::INVALID_EVENT);

  // This should spit out Dependency ID invalid error code
  error_code = scheduler->add_schedule(
    R"({"events":{}, "dependencies": {"dag-1":{}}})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_ID |
    ErrorCode::INVALID_DEPENDENCY);

  // This should spit out Series ID invalid error code
  error_code = scheduler->add_schedule(
    R"({"events":{}, "series":
      {"series-1":{"cron": "",
      "timezone":"UTC","occurrences":[]}}})"_json);
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_ID |
    ErrorCode::INVALID_SERIES);


  // Reset the scheduler
  scheduler.reset(new Scheduler());

  // Invalid dependency graph cyclic
  auto cyclic_dag_schedule_json_str =
    test_utils::init_json_from_file("json/cyclic_dag.json");
  error_code = scheduler->add_schedule(
    nlohmann::json::parse(cyclic_dag_schedule_json_str));
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_LOGIC |
    ErrorCode::INVALID_DEPENDENCY);

  // Invalid Series Logic (Empty series)
  auto series_empty_schedule_json_str =
    test_utils::init_json_from_file("json/series_empty.json");

  error_code = scheduler->add_schedule(
    nlohmann::json::parse(series_empty_schedule_json_str));
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_LOGIC |
    ErrorCode::INVALID_SERIES
  );

  // Invalid Series Logic (Invalid cron)
  auto invalid_cron_schedule_json_str =
    test_utils::init_json_from_file("json/invalid_cron.json");

  error_code = scheduler->add_schedule(
    nlohmann::json::parse(invalid_cron_schedule_json_str));
  std::cout << error_code.str() << std::endl;
  EXPECT_EQ(
    error_code.val,
    ErrorCode::FAILURE |
    ErrorCode::INVALID_LOGIC |
    ErrorCode::INVALID_SERIES
  );
}
