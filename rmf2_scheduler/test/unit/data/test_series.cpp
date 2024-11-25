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

#include "gtest/gtest.h"
#include "rmf2_scheduler/data/series.hpp"
#include "rmf2_scheduler/utils.hpp"
#include "../../utils/gtest_macros.hpp"
#include "./test_series_observer.hpp"

namespace rmf2_scheduler
{
namespace data
{

Series::UPtr make_preset_series()
{
  Time start_time = Time::from_localtime("Jan 2 10:15:00 2023");
  Time end_time = Time::from_localtime("Oct 3 10:15:00 2023");
  Series::UPtr series = std::make_unique<Series>(
    "series_1",
    "task",
    Occurrence{start_time, "event_1"},
    "0 15 10 ? * MON-FRI",    // 10:15 AM every Monday - Friday
    "Asia/Singapore",         // default timezone
    end_time
  );
  return series;
}
}  // namespace data
}  // namespace rmf2_scheduler


class TestSeries : public ::testing::Test
{
protected:
  void SetUp() override
  {
    using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
    utils::set_timezone("Asia/Singapore");
  }

  void TearDown() override
  {
  }
};

TEST_F(TestSeries, empty_series)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  Series empty_series;
  EXPECT_FALSE(empty_series);
  EXPECT_EQ(empty_series.cron(), "");
  EXPECT_TRUE(empty_series.occurrences().empty());
  EXPECT_THROW_EQ(
    empty_series.get_last_occurrence(),
    std::out_of_range("Series get_last_occurrence failed, series empty.")
  );
  EXPECT_THROW_EQ(
    empty_series.get_first_occurrence(),
    std::out_of_range("Series get_first_occurrence failed, series empty.")
  );
  EXPECT_EQ(empty_series.tz(), "UTC");
  EXPECT_EQ(empty_series.until().nanoseconds(), Time::max().nanoseconds());
  EXPECT_TRUE(empty_series.exception_ids().empty());
  EXPECT_THROW_EQ(
    empty_series.expand_until(Time::max()),
    std::out_of_range("Series expand_until failed, series empty.")
  );
}

TEST_F(TestSeries, preset_series)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  auto preset_series = make_preset_series();
  EXPECT_EQ(preset_series->cron(), "0 15 10 ? * MON-FRI");
  std::vector<Occurrence> expected_occurrence {
    Occurrence {
      Time::from_localtime("Jan 2 10:15:00 2023"),
      "event_1"
    }
  };
  EXPECT_EQ(preset_series->occurrences(), expected_occurrence);
  EXPECT_EQ(preset_series->tz(), "Asia/Singapore");
  EXPECT_EQ(
    preset_series->until().nanoseconds(),
    Time::from_localtime("Oct 3 10:15:00 2023").nanoseconds()
  );
  EXPECT_TRUE(preset_series->exception_ids().empty());
}

TEST_F(TestSeries, move)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  {
    // Move constructor
    auto preset_series = make_preset_series();
    auto preset_series_duplicate = make_preset_series();
    Series series(std::move(*preset_series));
    EXPECT_FALSE(*preset_series);
    EXPECT_TRUE(series);
    EXPECT_EQ(series, *preset_series_duplicate);
  }

  {
    // Move assignment constructor
    auto preset_series = make_preset_series();
    auto preset_series_duplicate = make_preset_series();
    Series series;
    series = std::move(*preset_series);
    EXPECT_FALSE(*preset_series);
    EXPECT_TRUE(series);
    EXPECT_EQ(series, *preset_series_duplicate);
  }
}

TEST_F(TestSeries, create_with_multiple_occurrences)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  auto series = make_preset_series();

  // Expand a week of occurrences
  Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
  series->expand_until(expand_until);

  // Invalid occurrences
  EXPECT_THROW_EQ(
    auto new_series = Series(
      series->id(),
      series->type(),
      std::vector<Occurrence>{},
      series->cron(),
      series->tz(),
      series->until()
    ),
    std::invalid_argument("Series create failed, no occurrences provided.")
  );

  // Invalid cron
  EXPECT_THROW_EQ(
    Series(
      series->id(),
      series->type(),
      series->occurrences(),
      "random_string",
      series->tz(),
      series->until()
    ),
    std::invalid_argument(
      "Series create failed, invalid cron [random_string]: "
      "cron expression must have six fields"
    )
  );

  // Mismatched occurrences
  {
    auto all_occurrences = series->occurrences();
    ASSERT_FALSE(all_occurrences.empty());
    all_occurrences.back().time -= Duration::from_seconds(3);
    EXPECT_THROW_EQ(
      Series(
        series->id(),
        series->type(),
        all_occurrences,
        series->cron(),
        series->tz(),
        series->until()
      ),
      std::invalid_argument(
        "Series create failed, invalid time [Jan 06 10:14:57 2023] "
        "doesn't match cron [0 15 10 ? * MON-FRI]."
      )
    );
  }

  // Success
  {
    // Create a new series based on description
    Series series_duplicate(
      series->id(),
      series->type(),
      series->occurrences(),
      series->cron(),
      series->tz(),
      series->until()
    );

    EXPECT_TRUE(*series == series_duplicate);
  }

  // Success with exception ids
  {
    // Create a exception in the series
    auto last_occurrence = series->get_last_occurrence();
    Time new_occurrence_time = last_occurrence.time - Duration::from_seconds(3);
    series->update_occurrence(last_occurrence.time, new_occurrence_time);

    Series series_duplicate(
      series->id(),
      series->type(),
      series->occurrences(),
      series->cron(),
      series->tz(),
      series->until(),
      {series->exception_ids().begin(), series->exception_ids().end()}
    );

    EXPECT_TRUE(*series == series_duplicate);
  }
}

TEST_F(TestSeries, is_bool)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  {
    Series empty_series;
    EXPECT_FALSE(empty_series);
  }

  {
    auto preset_series = make_preset_series();
    EXPECT_TRUE(*preset_series);
  }
}

TEST_F(TestSeries, get_occurrence)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  auto preset_series = make_preset_series();

  {  // By time
    EXPECT_THROW_EQ(
      preset_series->get_occurrence(Time(0)),
      std::out_of_range(
        "Series get_occurrence failed, time [Jan 01 07:30:00 1970] doesn't exist."
      )
    );

    Time time = Time::from_localtime("Jan 2 10:15:00 2023");
    std::string id = "event_1";
    auto occurrence = preset_series->get_occurrence(time);
    Occurrence expected_occurrence{time, id};
    EXPECT_EQ(occurrence, expected_occurrence);
  }

  {  // By ID
    EXPECT_THROW_EQ(
      preset_series->get_occurrence("event_2"),
      std::out_of_range(
        "Series get_occurrence failed, ID [event_2] doesn't exist."
      )
    );

    Time time = Time::from_localtime("Jan 2 10:15:00 2023");
    std::string id = "event_1";
    auto occurrence = preset_series->get_occurrence(id);
    Occurrence expected_occurrence{time, id};
    EXPECT_EQ(occurrence, expected_occurrence);
  }

  // Expand a week of occurrences
  Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
  preset_series->expand_until(expand_until);

  {  // First
    auto occurrence = preset_series->get_first_occurrence();

    Time time = Time::from_localtime("Jan 2 10:15:00 2023");
    std::string id = "event_1";
    Occurrence expected_occurrence{time, id};
    EXPECT_EQ(occurrence, expected_occurrence);
  }

  {  // Last
    auto occurrence = preset_series->get_last_occurrence();

    EXPECT_TRUE(occurrence.time == Time::from_localtime("Jan 06 10:15:00 2023"));
  }
}

TEST_F(TestSeries, expand_until)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  auto series = make_preset_series();

  {
    // Expand a week of occurrences
    Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
    series->expand_until(expand_until);

    auto all_occurrences = series->occurrences();

    ASSERT_EQ(all_occurrences.size(), 5lu);
    EXPECT_EQ(std::string(all_occurrences[0].time.to_localtime()), "Jan 02 10:15:00 2023");
    EXPECT_EQ(std::string(all_occurrences[1].time.to_localtime()), "Jan 03 10:15:00 2023");
    EXPECT_EQ(std::string(all_occurrences[2].time.to_localtime()), "Jan 04 10:15:00 2023");
    EXPECT_EQ(std::string(all_occurrences[3].time.to_localtime()), "Jan 05 10:15:00 2023");
    EXPECT_EQ(std::string(all_occurrences[4].time.to_localtime()), "Jan 06 10:15:00 2023");
  }

  {
    // Expand all occurrences
    series->expand_until(Time::max());
    auto all_occurrences = series->occurrences();

    // // Print out all occurrences
    // for (auto & occurrence : all_occurrences) {
    //   std::cout << "occurrence id: " << occurrence.id
    //     << ", time: " << occurrence.time.to_localtime() << std::endl;
    // }

    // Check if it is in total 197 occurrences
    EXPECT_EQ(all_occurrences.size(), 197lu);
  }
}

TEST_F(TestSeries, expand_until_with_observer)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  auto series = make_preset_series();
  auto series_observer = series->make_observer<TestSeriesObserver>();

  {
    // Expand a week of occurrences
    Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
    series->expand_until(expand_until);

    auto add_occurrences_record = series_observer->get_add_occurrence_record();
    ASSERT_EQ(add_occurrences_record.size(), 4lu);

    // Check reference occurrences
    EXPECT_EQ(add_occurrences_record[0].ref_occurrence, series->get_first_occurrence());
    EXPECT_EQ(add_occurrences_record[1].ref_occurrence, series->get_first_occurrence());
    EXPECT_EQ(add_occurrences_record[2].ref_occurrence, series->get_first_occurrence());
    EXPECT_EQ(add_occurrences_record[3].ref_occurrence, series->get_first_occurrence());

    // Check new occurrences
    EXPECT_EQ(
      std::string(add_occurrences_record[0].new_occurrence.time.to_localtime()),
      "Jan 03 10:15:00 2023"
    );
    EXPECT_EQ(
      std::string(add_occurrences_record[1].new_occurrence.time.to_localtime()),
      "Jan 04 10:15:00 2023"
    );
    EXPECT_EQ(
      std::string(add_occurrences_record[2].new_occurrence.time.to_localtime()),
      "Jan 05 10:15:00 2023"
    );
    EXPECT_EQ(
      std::string(add_occurrences_record[3].new_occurrence.time.to_localtime()),
      "Jan 06 10:15:00 2023"
    );
  }

  {
    // Expand another 2 days of occurrences
    Time expand_until = Time::from_localtime("Jan 11 10:14:59 2023");
    auto expected_ref_occurrence = series->get_last_occurrence();
    series->expand_until(expand_until);

    // Reference is updated
    auto add_occurrences_record = series_observer->get_add_occurrence_record();
    ASSERT_EQ(add_occurrences_record.size(), 6lu);
    EXPECT_EQ(add_occurrences_record[4].ref_occurrence, expected_ref_occurrence);
    EXPECT_EQ(add_occurrences_record[5].ref_occurrence, expected_ref_occurrence);
  }

  // Remove observer
  series->remove_observer(series_observer);

  {
    // Expand all occurrences
    series->expand_until(Time::max());
    auto all_occurrences = series->occurrences();

    // Observer no changes.
    auto add_occurrences_record = series_observer->get_add_occurrence_record();
    ASSERT_EQ(add_occurrences_record.size(), 6lu);
  }

  {
    // All other records are empty
    EXPECT_TRUE(series_observer->get_update_occurrence_record().empty());
    EXPECT_TRUE(series_observer->get_delete_occurrence_record().empty());
  }
}

TEST_F(TestSeries, update_id)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  auto series = make_preset_series();
  series->update_id("series_1_modified");

  EXPECT_EQ(series->id(), "series_1_modified");
}

TEST_F(TestSeries, update_occurrence)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  auto series = make_preset_series();

  // Expand to the next day
  series->expand_until(Time::from_localtime("Jan 3 10:15:00 2023"));

  {
    // Exception handling
    Time invalid_time = Time(0);

    EXPECT_THROW_EQ(
      series->update_occurrence(
        invalid_time,
        "event-1-modified"
      ),
      std::out_of_range(
        "Series update_occurrence failed, "
        "time [Jan 01 07:30:00 1970] doesn't exist."
      )
    );

    EXPECT_THROW_EQ(
      series->update_occurrence(
        invalid_time,
        Time::from_localtime("Jan 2 18:15:00 2023")
      ),
      std::out_of_range(
        "Series update_occurrence failed, "
        "time [Jan 01 07:30:00 1970] doesn't exist."
      )
    );
  }

  {
    // Update the ID of the first occurrence
    Time time_to_update = Time::from_localtime("Jan 2 10:15:00 2023");

    series->update_occurrence(time_to_update, "event-1-modified");

    // Check ID updated
    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 2lu);
    EXPECT_EQ(all_occurrences[0].id, "event-1-modified");
  }

  {
    // Update the time of the first occurrence
    Time time_to_update = Time::from_localtime("Jan 2 10:15:00 2023");

    series->update_occurrence(
      time_to_update,
      Time::from_localtime("Jan 2 18:15:00 2023")
    );

    // Check time updated
    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 2lu);
    EXPECT_EQ(std::string(all_occurrences[0].time.to_localtime()), "Jan 02 18:15:00 2023");

    // Check exceptions added
    auto all_exceptions = series->exception_ids_ordered();
    ASSERT_EQ(all_exceptions.size(), 1lu);
    EXPECT_EQ(all_exceptions[0], "event-1-modified");
  }

  {
    // Update the ID of the last occurrence
    Time time_to_update = Time::from_localtime("Jan 3 10:15:00 2023");
    series->update_occurrence(time_to_update, "event-2-modified");

    // No expansion
    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 2lu);

    // Check ID updated
    EXPECT_EQ(all_occurrences[1].id, "event-2-modified");
  }

  {
    // Update the time of the last occurrence
    // series should automatically expand one more
    Time time_to_update = Time::from_localtime("Jan 3 10:15:00 2023");
    series->update_occurrence(
      time_to_update,
      Time::from_localtime("Jan 3 18:16:00 2023")
    );

    // Check series expanded
    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 3lu);

    // Check time updated
    EXPECT_EQ(std::string(all_occurrences[1].time.to_localtime()), "Jan 03 18:16:00 2023");

    // Check the time of the last event added
    EXPECT_EQ(std::string(all_occurrences[2].time.to_localtime()), "Jan 04 10:15:00 2023");

    // Check exceptions added
    auto all_exceptions = series->exception_ids_ordered();
    ASSERT_EQ(all_exceptions.size(), 2lu);
    EXPECT_EQ(all_exceptions[1], "event-2-modified");
  }

  {
    // Update the ID of an exception
    Time time_to_update = Time::from_localtime("Jan 3 18:16:00 2023");
    series->update_occurrence(
      time_to_update,
      "event-2-modified-x2"
    );

    // Check exception ID updated
    auto all_exceptions = series->exception_ids_ordered();
    ASSERT_EQ(all_exceptions.size(), 2lu);
    EXPECT_EQ(all_exceptions[1], "event-2-modified-x2");
  }

  {
    // Delete the exception
    series->delete_occurrence(Time::from_localtime("Jan 3 18:16:00 2023"));

    // Check if the exception is deleted
    auto all_exceptions = series->exception_ids_ordered();
    ASSERT_EQ(series->exception_ids().size(), 1lu);
    EXPECT_EQ(all_exceptions[0], "event-1-modified");
  }
}

TEST_F(TestSeries, update_occurrence_with_observer)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  auto series = make_preset_series();

  // Expand to the next day
  series->expand_until(Time::from_localtime("Jan 3 10:15:01 2023"));

  auto series_observer = series->make_observer<TestSeriesObserver>();
  {
    // Update the time of the first occurrence
    Time time_to_update = Time::from_localtime("Jan 2 10:15:00 2023");

    series->update_occurrence(
      time_to_update,
      Time::from_localtime("Jan 2 18:15:00 2023")
    );
    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 2lu);

    // Check observer update record
    auto update_occurrences_record = series_observer->get_update_occurrence_record();
    ASSERT_EQ(update_occurrences_record.size(), 1lu);
    EXPECT_TRUE(update_occurrences_record[0].old_occurrence.time == time_to_update);
    EXPECT_EQ(
      std::string(update_occurrences_record[0].new_time.to_localtime()),
      "Jan 02 18:15:00 2023"
    );
    auto records = series_observer->get_add_occurrence_record();
    EXPECT_TRUE(series_observer->get_add_occurrence_record().empty());
  }

  {
    // Update the time of the last occurrence
    // series should automatically expand one more
    Time time_to_update = Time::from_localtime("Jan 3 10:15:00 2023");
    series->update_occurrence(
      time_to_update,
      Time::from_localtime("Jan 3 18:16:00 2023")
    );

    // Check observer update and add record
    auto add_occurrences_record = series_observer->get_add_occurrence_record();
    ASSERT_EQ(add_occurrences_record.size(), 1lu);
    EXPECT_TRUE(add_occurrences_record[0].ref_occurrence.time == time_to_update);
    EXPECT_EQ(
      std::string(add_occurrences_record[0].new_occurrence.time.to_localtime()),
      "Jan 04 10:15:00 2023"
    );

    auto update_occurrences_record = series_observer->get_update_occurrence_record();
    ASSERT_EQ(update_occurrences_record.size(), 2lu);
    EXPECT_TRUE(update_occurrences_record[1].old_occurrence.time == time_to_update);
    EXPECT_EQ(
      std::string(update_occurrences_record[1].new_time.to_localtime()),
      "Jan 03 18:16:00 2023"
    );

    // Check update is done after add
    EXPECT_TRUE(add_occurrences_record[0].record_time < update_occurrences_record[1].record_time);
  }

  EXPECT_TRUE(series_observer->get_delete_occurrence_record().empty());
}

TEST_F(TestSeries, update_cron_from_exceptions)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  auto series = make_preset_series();

  // Expand a week of occurrences
  Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
  series->expand_until(expand_until);

  // Invalid time
  EXPECT_THROW_EQ(
    series->update_cron_from(
      Time(0),
      Time::from_localtime("Jan 4 18:32:12 2023"),
      "12 32 18 ? * MON,TUE,WED,FRI",
      "Asia/Singapore"
    ),
    std::out_of_range(
      "Series update_cron_from failed, "
      "time [Jan 01 07:30:00 1970] doesn't exist."
    )
  );

  // Make an exception for one of the occurrence
  series->update_occurrence(
    Time::from_localtime("Jan 4 10:15:00 2023"),
    Time::from_localtime("Jan 4 10:16:00 2023")
  );

  // Cannot change cron from exception
  EXPECT_THROW_EQ(
    series->update_cron_from(
      Time::from_localtime("Jan 4 10:16:00 2023"),
      Time::from_localtime("Jan 4 18:32:12 2023"),
      "12 32 18 ? * MON,TUE,WED,FRI",
      "Asia/Singapore"
    ),
    std::invalid_argument(
      "Series update_cron_from failed, "
      "time [Jan 04 10:16:00 2023] is already an exception."
    )
  );

  // Invalid cron
  EXPECT_THROW_EQ(
    series->update_cron_from(
      Time::from_localtime("Jan 3 10:15:00 2023"),
      Time::from_localtime("Jan 3 18:32:12 2023"),
      "random_string",
      "Asia/Singapore"),
    std::invalid_argument(
      "Series update_cron_from failed, invalid cron [random_string]: "
      "cron expression must have six fields"
    )
  );

  // New time doesn't match cron
  EXPECT_THROW_EQ(
    series->update_cron_from(
      Time::from_localtime("Jan 3 10:15:00 2023"),
      Time::from_localtime("Jan 3 18:32:13 2023"),
      "12 32 18 ? * MON,TUE,WED,FRI",
      "Asia/Singapore"
    ),
    std::invalid_argument(
      "Series update_cron_from failed, "
      "invalid time [Jan 03 18:32:13 2023] doesn't match "
      "cron [12 32 18 ? * MON,TUE,WED,FRI]."
    )
  );
}

TEST_F(TestSeries, update_cron_from)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  auto series = make_preset_series();
  // Expand a week of occurrences
  Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
  series->expand_until(expand_until);

  {
    // Cron no changes
    Series series_duplicate(
      series->id(),
      series->type(),
      series->occurrences(),
      series->cron(),
      series->tz(),
      series->until()
    );

    // No changes
    series->update_cron_from(
      Time::from_localtime("Jan 4 10:15:00 2023"),
      Time::from_localtime("Jan 4 10:15:00 2023"),
      "0 15 10 ? * MON-FRI",    // 10:15 AM every Monday - Friday
      "Asia/Singapore"
    );

    // series no changes
    EXPECT_EQ(*series, series_duplicate);
  }

  // Change the cron from the Wednesday event
  series->update_cron_from(
    Time::from_localtime("Jan 4 10:15:00 2023"),
    Time::from_localtime("Jan 4 18:32:12 2023"),
    "12 32 18 ? * MON,TUE,WED,FRI",
    "Asia/Singapore");

  {
    // Check if occurrences are all updated.
    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 5lu);
    EXPECT_EQ(std::string(all_occurrences[0].time.to_localtime()), "Jan 02 10:15:00 2023");
    EXPECT_EQ(std::string(all_occurrences[1].time.to_localtime()), "Jan 03 10:15:00 2023");
    EXPECT_EQ(std::string(all_occurrences[2].time.to_localtime()), "Jan 04 18:32:12 2023");
    EXPECT_EQ(std::string(all_occurrences[3].time.to_localtime()), "Jan 06 18:32:12 2023");
    EXPECT_EQ(std::string(all_occurrences[4].time.to_localtime()), "Jan 09 18:32:12 2023");

    // Check if all previous occurrences are considered exception
    auto all_exceptions = series->exception_ids();
    EXPECT_EQ(all_exceptions.size(), 2lu);
    EXPECT_TRUE(all_exceptions.find(all_occurrences[0].id) != all_exceptions.end());
    EXPECT_TRUE(all_exceptions.find(all_occurrences[1].id) != all_exceptions.end());
  }

  // Change the Friday occurrence
  series->update_occurrence(
    Time::from_localtime("Jan 6 18:32:12 2023"),
    Time::from_localtime("Jan 7 18:16:00 2023")
  );

  // Expand for another week
  series->expand_until(
    Time::from_localtime("Jan 13 23:59:59 2023")
  );

  {
    // Check if all future occurrences are updated based on the new cron.
    auto all_occurrences = series->occurrences();

    ASSERT_EQ(all_occurrences.size(), 8lu);
    EXPECT_EQ(std::string(all_occurrences[3].time.to_localtime()), "Jan 07 18:16:00 2023");
    EXPECT_EQ(std::string(all_occurrences[4].time.to_localtime()), "Jan 09 18:32:12 2023");
    EXPECT_EQ(std::string(all_occurrences[5].time.to_localtime()), "Jan 10 18:32:12 2023");
    EXPECT_EQ(std::string(all_occurrences[6].time.to_localtime()), "Jan 11 18:32:12 2023");
    EXPECT_EQ(std::string(all_occurrences[7].time.to_localtime()), "Jan 13 18:32:12 2023");

    // Check if new exceptions is added
    auto all_exceptions = series->exception_ids();
    EXPECT_EQ(all_exceptions.size(), 3lu);
    EXPECT_TRUE(all_exceptions.find(all_occurrences[3].id) != all_exceptions.end());
  }

  // Change cron again from the Wednesday event
  series->update_cron_from(
    Time::from_localtime("Jan 4 18:32:12 2023"),
    Time::from_localtime("Jan 4 19:00:18 2023"),
    "18 00 19 ? * TUE,WED,FRI",
    "Asia/Singapore"
  );

  {
    auto all_occurrences = series->occurrences();

    // Check if all cron occurrences are updated
    EXPECT_EQ(all_occurrences.size(), 8lu);
    EXPECT_EQ(std::string(all_occurrences[2].time.to_localtime()), "Jan 04 19:00:18 2023");
    EXPECT_EQ(std::string(all_occurrences[3].time.to_localtime()), "Jan 07 18:16:00 2023");
    EXPECT_EQ(std::string(all_occurrences[4].time.to_localtime()), "Jan 10 19:00:18 2023");
    EXPECT_EQ(std::string(all_occurrences[5].time.to_localtime()), "Jan 11 19:00:18 2023");
    EXPECT_EQ(std::string(all_occurrences[6].time.to_localtime()), "Jan 13 19:00:18 2023");
    EXPECT_EQ(std::string(all_occurrences[7].time.to_localtime()), "Jan 17 19:00:18 2023");

    // Check if all previous occurrences & exceptions are considered exception
    auto all_exceptions = series->exception_ids();
    EXPECT_EQ(all_exceptions.size(), 3lu);
    EXPECT_TRUE(all_exceptions.find(all_occurrences[0].id) != all_exceptions.end());
    EXPECT_TRUE(all_exceptions.find(all_occurrences[1].id) != all_exceptions.end());
    EXPECT_TRUE(all_exceptions.find(all_occurrences[3].id) != all_exceptions.end());
  }
}

TEST_F(TestSeries, update_cron_from_with_observer)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  auto series = make_preset_series();
  // Expand a week of occurrences
  Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
  series->expand_until(expand_until);

  auto series_observer = series->make_observer<TestSeriesObserver>();

  // Change the cron from the Wednesday event
  series->update_cron_from(
    Time::from_localtime("Jan 4 10:15:00 2023"),
    Time::from_localtime("Jan 4 18:32:12 2023"),
    "12 32 18 ? * MON,TUE,WED,FRI",
    "Asia/Singapore");

  // Check the observer update record
  auto update_occurrences_record = series_observer->get_update_occurrence_record();
  ASSERT_EQ(update_occurrences_record.size(), 3lu);
  EXPECT_EQ(
    std::string(update_occurrences_record[0].old_occurrence.time.to_localtime()),
    "Jan 04 10:15:00 2023"
  );
  EXPECT_EQ(
    std::string(update_occurrences_record[0].new_time.to_localtime()),
    "Jan 04 18:32:12 2023"
  );
  EXPECT_EQ(
    std::string(update_occurrences_record[1].old_occurrence.time.to_localtime()),
    "Jan 05 10:15:00 2023"
  );
  EXPECT_EQ(
    std::string(update_occurrences_record[1].new_time.to_localtime()),
    "Jan 06 18:32:12 2023"
  );
  EXPECT_EQ(
    std::string(update_occurrences_record[2].old_occurrence.time.to_localtime()),
    "Jan 06 10:15:00 2023"
  );
  EXPECT_EQ(
    std::string(update_occurrences_record[2].new_time.to_localtime()),
    "Jan 09 18:32:12 2023"
  );

  EXPECT_TRUE(series_observer->get_add_occurrence_record().empty());
  EXPECT_TRUE(series_observer->get_delete_occurrence_record().empty());
}

TEST_F(TestSeries, update_until)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  {  // No deletion needed
    auto series = make_preset_series();

    // Expand a week of occurrences
    Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
    series->expand_until(expand_until);

    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 5lu);

    // Update until, no delete should occur
    series->update_until(Time::from_localtime("Jan 6 10:15:00 2023"));
    all_occurrences = series->occurrences();

    EXPECT_EQ(all_occurrences.size(), 5lu);
  }

  {  // Deletion needed
    auto series = make_preset_series();

    // Expand a week of occurrences
    Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
    series->expand_until(expand_until);

    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 5lu);

    // Update until, all occurrences after should be deleted
    series->update_until(Time::from_localtime("Jan 3 10:15:01 2023"));
    all_occurrences = series->occurrences();

    EXPECT_EQ(all_occurrences.size(), 2lu);
  }

  {  // Deletion needed, with exception
    auto series = make_preset_series();

    // Expand a week of occcurences
    Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
    series->expand_until(expand_until);

    // Make an exception for the second last occurrence
    series->update_occurrence(
      Time::from_localtime("Jan 5 10:15:00 2023"),
      Time::from_localtime("Jan 5 10:16:00 2023")
    );

    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 5lu);

    auto all_exceptions = series->exception_ids_ordered();
    ASSERT_EQ(all_exceptions.size(), 1lu);

    // Update until, all occurrences after should be deleted
    series->update_until(Time::from_localtime("Jan 3 10:15:01 2023"));
    all_occurrences = series->occurrences();
    all_exceptions = series->exception_ids_ordered();

    ASSERT_EQ(all_occurrences.size(), 2lu);
    ASSERT_TRUE(all_exceptions.empty());
  }
}

TEST_F(TestSeries, update_until_with_observer)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  auto series = make_preset_series();

  // Expand a week of occurrences
  Time expand_until = Time::from_localtime("Jan 9 10:14:59 2023");
  series->expand_until(expand_until);
  auto all_occurrences = series->occurrences();
  ASSERT_EQ(all_occurrences.size(), 5lu);
  std::vector<std::string> ids_to_delete {
    all_occurrences[2].id,
    all_occurrences[3].id,
    all_occurrences[4].id,
  };

  auto series_observer = series->make_observer<TestSeriesObserver>();

  // Update until, all occurrences after should be deleted
  series->update_until(Time::from_localtime("Jan 3 10:15:01 2023"));

  // Check observer
  auto delete_occurrences_record = series_observer->get_delete_occurrence_record();

  ASSERT_EQ(delete_occurrences_record.size(), 3lu);
  EXPECT_EQ(delete_occurrences_record[0].occurrence.id, ids_to_delete[0]);
  EXPECT_EQ(delete_occurrences_record[1].occurrence.id, ids_to_delete[1]);
  EXPECT_EQ(delete_occurrences_record[2].occurrence.id, ids_to_delete[2]);

  EXPECT_TRUE(series_observer->get_add_occurrence_record().empty());
  EXPECT_TRUE(series_observer->get_update_occurrence_record().empty());
}

TEST_F(TestSeries, delete_occurrence)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  auto series = make_preset_series();

  // Expand to the next day
  series->expand_until(Time::from_localtime("Jan 3 10:15:00 2023"));

  // Invalid time
  EXPECT_THROW_EQ(
    series->delete_occurrence(Time(0)),
    std::out_of_range(
      "Series delete_occurrence failed, "
      "time [Jan 01 07:30:00 1970] doesn't exist."
    )
  );

  {
    // Delete the first one
    Time time_to_delete = Time::from_localtime("Jan 2 10:15:00 2023");
    series->delete_occurrence(time_to_delete);

    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 1lu);
    EXPECT_EQ(std::string(all_occurrences[0].time.to_localtime()), "Jan 03 10:15:00 2023");
  }

  {
    // Delete the last occurrence, series should automatically expand one more
    Time time_to_delete = Time::from_localtime("Jan 3 10:15:00 2023");
    series->delete_occurrence(time_to_delete);

    auto all_occurrences = series->occurrences();

    // Two deleted, last occurrence deleted would expand one amounting to one
    ASSERT_EQ(all_occurrences.size(), 1lu);
    EXPECT_EQ(std::string(all_occurrences[0].time.to_localtime()), "Jan 04 10:15:00 2023");
  }

  {
    // Update the until time
    series->update_until(Time::from_localtime("Jan 5 12:15:00 2023"));

    // Expand till the end
    series->expand_until(Time::max());

    // Delete last occurrence
    Time time_to_delete = Time::from_localtime("Jan 5 10:15:00 2023");
    series->delete_occurrence(time_to_delete);

    auto all_occurrences = series->occurrences();
    ASSERT_EQ(all_occurrences.size(), 1lu);
    EXPECT_EQ(std::string(all_occurrences[0].time.to_localtime()), "Jan 04 10:15:00 2023");

    // Check until is updated
    EXPECT_EQ(std::string(series->until().to_localtime()), "Jan 05 10:14:59 2023");
  }
}

TEST_F(TestSeries, delete_occurrence_with_observer)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  auto series = make_preset_series();

  // Expand to the next day
  series->expand_until(Time::from_localtime("Jan 3 10:15:00 2023"));

  auto series_observer = series->make_observer<TestSeriesObserver>();

  {
    // Delete the first one
    Time time_to_delete = Time::from_localtime("Jan 2 10:15:00 2023");
    std::string id_to_delete = series->get_first_occurrence().id;
    series->delete_occurrence(time_to_delete);

    // Check observer
    auto delete_occurrences_record = series_observer->get_delete_occurrence_record();

    ASSERT_EQ(delete_occurrences_record.size(), 1lu);
    EXPECT_EQ(delete_occurrences_record[0].occurrence.id, id_to_delete);
    EXPECT_TRUE(series_observer->get_add_occurrence_record().empty());
    EXPECT_TRUE(series_observer->get_update_occurrence_record().empty());
  }

  {
    // Delete the last occurrence, series should automatically expand one more
    Time time_to_delete = Time::from_localtime("Jan 3 10:15:00 2023");
    std::string id_to_delete = series->get_first_occurrence().id;
    series->delete_occurrence(time_to_delete);

    // Check observer, both add and delete record should be updated
    auto add_occurrences_record = series_observer->get_add_occurrence_record();
    ASSERT_EQ(add_occurrences_record.size(), 1lu);
    EXPECT_EQ(add_occurrences_record[0].ref_occurrence.id, id_to_delete);

    auto delete_occurrences_record = series_observer->get_delete_occurrence_record();
    ASSERT_EQ(delete_occurrences_record.size(), 2lu);
    EXPECT_EQ(delete_occurrences_record[1].occurrence.id, id_to_delete);

    // Add happens before delete
    EXPECT_TRUE(add_occurrences_record[0].record_time < delete_occurrences_record[1].record_time);

    EXPECT_TRUE(series_observer->get_update_occurrence_record().empty());
  }
}
