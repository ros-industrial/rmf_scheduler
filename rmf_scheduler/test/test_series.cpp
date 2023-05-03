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
#include "rmf_scheduler/series.hpp"
#include "rmf_scheduler/system_time_utils.hpp"

class TestSeries : public ::testing::Test
{
protected:
  void SetUp() override
  {
    using namespace rmf_scheduler;  // NOLINT(build/namespaces)
    uint64_t start_time = utils::from_localtime("Jan 2 10:15:00 2023");
    uint64_t end_time = utils::from_localtime("Oct 3 10:15:00 2023");
    series_ = Series(
      "event-1",
      start_time,
      "0 15 10 ? * MON-FRI",    // 10:15 AM every Monday - Friday
      end_time,
      "event-");
  }

  void TearDown() override
  {
  }

  rmf_scheduler::Series series_;
};


TEST_F(TestSeries, BasicExpand)
{
  // Expand all occurrences
  series_.expand_until(UINT64_MAX);
  auto description = series_.description();

  // Print out all occurrences
  // for (auto & occurrence : description.occurrences) {
  //   std::cout << "occurrence id: " << occurrence.id
  //     << ", time: " << utils::to_localtime(occurrence.time) << std::endl;
  // }

  // Check if it is in total 197 occurrences
  EXPECT_EQ(static_cast<int>(description.occurrences.size()), 197);

  // Check their prefixes
  for (auto & occurrence : description.occurrences) {
    EXPECT_NE(occurrence.id.rfind("event-", 6), std::string::npos);
  }
}

TEST_F(TestSeries, BasicDescriptionReload)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  uint64_t first_expand_until = utils::from_localtime("Jan 9 10:14:59 2023");

  // Expand a week of occcurances
  series_.expand_until(first_expand_until);

  // Export description
  auto description = series_.description();

  // Create a new series based on description
  Series new_series(description);
  auto new_description = new_series.description();

  // Print out all occurrences
  // for (auto & occurrence : new_description.occurrences) {
  //   std::cout << "occurrence id: " << occurrence.id
  //     << ", time: " << utils::to_localtime(occurrence.time) << std::endl;
  // }

  // Check if it is in total 5 occurrences
  EXPECT_EQ(static_cast<int>(new_description.occurrences.size()), 5);

  // Check their prefixes
  for (auto & occurrence : new_description.occurrences) {
    EXPECT_NE(occurrence.id.rfind("event-", 6), std::string::npos);
  }

  new_series.set_id_prefix("new-event-");
  // Expand for another week
  uint64_t second_expand_until = utils::from_localtime("Jan 16 10:14:59 2023");
  new_series.expand_until(second_expand_until);
  new_description = new_series.description();

  // Print out all occurrences
  for (auto & occurrence : new_description.occurrences) {
    std::cout << "occurrence id: " << occurrence.id
              << ", time: " << utils::to_localtime(occurrence.time) << std::endl;
  }

  // Check their prefixes
  for (auto & occurrence : new_description.occurrences) {
    if (occurrence.time <= first_expand_until) {
      EXPECT_NE(occurrence.id.rfind("event-", 6), std::string::npos);
    } else {
      EXPECT_NE(occurrence.id.rfind("new-event-", 10), std::string::npos);
    }
  }
}

TEST_F(TestSeries, SafeDelete)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Expand to the next day
  series_.expand_until(utils::from_localtime("Jan 3 10:15:00 2023"));

  // Delete the first one
  uint64_t time_to_delete = utils::from_localtime("Jan 2 10:15:00 2023");
  series_.delete_occurrence(time_to_delete);

  // Delete the last occurrence, series should automatically expand one more
  time_to_delete = utils::from_localtime("Jan 3 10:15:00 2023");
  series_.delete_occurrence(time_to_delete);

  auto description = series_.description();

  // Print out all occurrences
  for (auto & occurrence : description.occurrences) {
    std::cout << "occurrence id: " << occurrence.id
              << ", time: " << utils::to_localtime(occurrence.time) << std::endl;
  }
  // Two deleted, last occurrence deleted would expand one amounting to one
  EXPECT_EQ(static_cast<int>(description.occurrences.size()), 1);
}

TEST_F(TestSeries, UpdateOccurrenceTime)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Expand to the next day
  series_.expand_until(utils::from_localtime("Jan 3 10:15:00 2023"));

  // Update the first one
  uint64_t time_to_update = utils::from_localtime("Jan 2 10:15:00 2023");

  series_.update_occurrence_id(time_to_update, "event-1-modified");
  series_.update_occurrence_time(
    time_to_update,
    utils::from_localtime("Jan 2 18:15:00 2023"));

  // Update the last occurrence, series should automatically expand one more
  time_to_update = utils::from_localtime("Jan 3 10:15:00 2023");
  series_.update_occurrence_id(time_to_update, "event-2-modified");
  series_.update_occurrence_time(
    time_to_update,
    utils::from_localtime("Jan 3 18:16:00 2023"));

  auto description = series_.description();

  EXPECT_EQ(static_cast<int>(description.occurrences.size()), 3);
  EXPECT_EQ(static_cast<int>(description.exception_ids.size()), 2);

  // Print out all occurrences
  for (auto & occurrence : description.occurrences) {
    std::cout << "occurrence id: " << occurrence.id
              << ", time: " << utils::to_localtime(occurrence.time) << std::endl;
  }

  // Delete the exception
  series_.delete_occurrence(utils::from_localtime("Jan 3 18:16:00 2023"));
  description = series_.description();

  // Check if the exception is deleted
  EXPECT_EQ(static_cast<int>(description.occurrences.size()), 2);
  EXPECT_EQ(static_cast<int>(description.exception_ids.size()), 1);
}

TEST_F(TestSeries, UpdateCron)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  uint64_t first_expand_until = utils::from_localtime("Jan 9 10:14:59 2023");

  // Expand a week of occcurances
  series_.expand_until(first_expand_until);

  // Change the cron from the Wednesday event
  series_.update_cron_from(
    utils::from_localtime("Jan 4 10:15:00 2023"),
    utils::from_localtime("Jan 4 18:32:12 2023"),
    "12 32 18 ? * MON,TUE,WED,FRI");

  auto description = series_.description();

  // Print out all occurrences
  for (auto & occurrence : description.occurrences) {
    std::cout << "occurrence id: " << occurrence.id
              << ", time: " << utils::to_localtime(occurrence.time) << std::endl;
  }

  // Check if all cron occurrences are updated
  EXPECT_EQ(static_cast<int>(description.occurrences.size()), 5);

  // Check if all previous occurrences are considered exception
  EXPECT_EQ(static_cast<int>(description.exception_ids.size()), 2);

  // Change the Friday occurrence
  series_.update_occurrence_time(
    utils::from_localtime("Jan 6 18:32:12 2023"),
    utils::from_localtime("Jan 7 18:16:00 2023"));

  // Expand for another week
  series_.expand_until(
    utils::from_localtime("Jan 13 23:59:59 2023"));

  // Change cron again from the Wednesday event
  series_.update_cron_from(
    utils::from_localtime("Jan 4 00:00:00 2023"),
    utils::from_localtime("Jan 4 19:00:18 2023"),
    "18 00 19 ? * TUE,WED,FRI");

  description = series_.description();

  // Print out all occurrences
  for (auto & occurrence : description.occurrences) {
    std::cout << "occurrence id: " << occurrence.id
              << ", time: " << utils::to_localtime(occurrence.time) << std::endl;
  }

  // Check if all cron occurrences are updated
  EXPECT_EQ(static_cast<int>(description.occurrences.size()), 8);

  // Check if all previous occurrences & exceptions are considered exception
  EXPECT_EQ(static_cast<int>(description.exception_ids.size()), 3);
}
