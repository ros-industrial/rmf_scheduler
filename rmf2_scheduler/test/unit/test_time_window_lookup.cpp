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

#include "rmf2_scheduler/time_window_lookup.hpp"
#include "../utils/gtest_macros.hpp"

std::vector<rmf2_scheduler::TimeWindowLookup::Entry> create_preset_entries()
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using TimeWindowLookup = rmf2_scheduler::TimeWindowLookup;  // NOLINT(build/namespaces)

  TimeWindowLookup::Entry entry1 {
    "entry1",
    "event",
    {
      Time::from_localtime("Jan 01 00:00:00 2023"),
      Time::from_localtime("Jan 01 00:03:00 2023"),
    }
  };

  TimeWindowLookup::Entry entry2 {
    "entry2",
    "event",
    {
      Time::from_localtime("Jan 01 00:00:00 2023"),
      Time::from_localtime("Jan 01 00:01:00 2023"),
    }
  };

  TimeWindowLookup::Entry entry3 {
    "entry3",
    "event",
    {
      Time::from_localtime("Jan 01 00:01:00 2023"),
      Time::from_localtime("Jan 01 00:03:00 2023"),
    }
  };
  TimeWindowLookup::Entry entry4 {
    "entry4",
    "event",
    {
      Time::from_localtime("Jan 01 00:02:00 2023"),
      Time::from_localtime("Jan 01 00:04:00 2023")
    }
  };

  TimeWindowLookup::Entry entry5 {
    "entry5",
    "event",
    {
      Time::from_localtime("Jan 01 00:00:00 2023"),
      Time::from_localtime("Jan 01 00:06:00 2023")
    }
  };

  TimeWindowLookup::Entry entry6 {
    "entry6",
    "event",
    {
      Time::from_localtime("Jan 01 00:03:00 2023"),
      Time::from_localtime("Jan 01 00:06:00 2023")
    }
  };

  TimeWindowLookup::Entry entry7 {
    "entry7",
    "event",
    {
      Time::from_localtime("Jan 01 00:00:00 2023"),
      Time::from_localtime("Jan 01 00:02:00 2023"),
    }
  };

  TimeWindowLookup::Entry entry8 {
    "entry8",
    "event",
    {
      Time::from_localtime("Jan 01 00:04:00 2023"),
      Time::from_localtime("Jan 01 00:06:00 2023")
    }
  };

  TimeWindowLookup::Entry entry9 {
    "entry9",
    "event",
    {
      Time::from_localtime("Jan 01 00:05:00 2023"),
      Time::from_localtime("Jan 01 00:06:00 2023")
    }
  };

  TimeWindowLookup::Entry entry10 {
    "entry10",
    "event",
    {
      Time::from_localtime("Jan 01 00:02:00 2023"),
      Time::from_localtime("Jan 01 00:03:00 2023")
    }
  };

  return {
    entry1, entry2, entry3, entry4, entry5,
    entry6, entry7, entry8, entry9, entry10
  };
}

class TestTimeWindowLookup : public ::testing::Test
{
};

TEST_F(TestTimeWindowLookup, entry_crud) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using TimeWindowLookup = rmf2_scheduler::TimeWindowLookup;  // NOLINT(build/namespaces)

  // Create
  TimeWindowLookup lookup;
  TimeWindowLookup::Entry entry1 {
    "entry1",
    "event",
    {
      Time::from_localtime("Jan 01 00:00:00 2023"),
      Time::from_localtime("Jan 01 00:03:00 2023"),
    }
  };
  TimeWindowLookup::Entry entry2 {
    "entry2",
    "event",
    {
      Time::from_localtime("Jan 01 00:01:00 2023"),
      Time::from_localtime("Jan 01 00:04:00 2023")
    }
  };
  lookup.add_entry(entry1);
  lookup.add_entry(entry2);

  // Read
  auto entry1_found = lookup.get_entry("entry1");
  EXPECT_TRUE(entry1_found == entry1);

  auto entry2_found = lookup.get_entry("entry2");
  EXPECT_TRUE(entry2_found == entry2);

  // Delete
  lookup.remove_entry("entry1");
  EXPECT_THROW_EQ(
    lookup.get_entry("entry1"),
    std::invalid_argument("ID doesn't exist in lookup"));

  lookup.remove_entry("entry2");
  EXPECT_THROW_EQ(
    lookup.get_entry("entry2"),
    std::invalid_argument("ID doesn't exist in lookup"));

  EXPECT_THROW_EQ(
    lookup.remove_entry("entry1"),
    std::invalid_argument("ID doesn't exist in lookup"));
}

TEST_F(TestTimeWindowLookup, lookup) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using TimeWindowLookup = rmf2_scheduler::TimeWindowLookup;  // NOLINT(build/namespaces)

  {
    // Both hard
    auto preset_entries = create_preset_entries();
    TimeWindowLookup time_window_lookup;
    for (auto & entry : preset_entries) {
      time_window_lookup.add_entry(entry);
    }

    auto result = time_window_lookup.lookup(
      TimeWindow{
      Time::from_localtime("Jan 01 00:01:00 2023"),
      Time::from_localtime("Jan 01 00:04:00 2023")
    },
      false,
      false
    );

    // for (auto & entry : result) {
    //   std::cout << entry.id << ": "
    //     << entry.time_window.start.to_localtime() << " - "
    //     << entry.time_window.end.to_localtime() << std::endl;
    // }

    // Check result
    ASSERT_EQ(result.size(), 3lu);

    EXPECT_TRUE(result[0] == preset_entries[2]);
    EXPECT_TRUE(result[1] == preset_entries[3]);
    EXPECT_TRUE(result[2] == preset_entries[9]);
  }

  {
    // soft hard
    auto preset_entries = create_preset_entries();
    TimeWindowLookup time_window_lookup;
    for (auto & entry : preset_entries) {
      time_window_lookup.add_entry(entry);
    }

    auto result = time_window_lookup.lookup(
      TimeWindow{
      Time::from_localtime("Jan 01 00:01:00 2023"),
      Time::from_localtime("Jan 01 00:04:00 2023")
    },
      true,
      false
    );

    // for (auto & entry : result) {
    //   std::cout << entry.id << ": "
    //     << entry.time_window.start.to_localtime() << " - "
    //     << entry.time_window.end.to_localtime() << std::endl;
    // }

    // Check result
    ASSERT_EQ(result.size(), 6lu);

    EXPECT_TRUE(result[0] == preset_entries[1]);
    EXPECT_TRUE(result[1] == preset_entries[6]);
    EXPECT_TRUE(result[2] == preset_entries[0]);
    EXPECT_TRUE(result[3] == preset_entries[2]);
    EXPECT_TRUE(result[4] == preset_entries[9]);
    EXPECT_TRUE(result[5] == preset_entries[3]);
  }

  {
    // hard soft
    auto preset_entries = create_preset_entries();
    TimeWindowLookup time_window_lookup;
    for (auto & entry : preset_entries) {
      time_window_lookup.add_entry(entry);
    }

    auto result = time_window_lookup.lookup(
      TimeWindow{
      Time::from_localtime("Jan 01 00:01:00 2023"),
      Time::from_localtime("Jan 01 00:04:00 2023")
    },
      false,
      true
    );

    // for (auto & entry : result) {
    //   std::cout << entry.id << ": "
    //     << entry.time_window.start.to_localtime() << " - "
    //     << entry.time_window.end.to_localtime() << std::endl;
    // }

    // Check result
    ASSERT_EQ(result.size(), 5lu);

    EXPECT_TRUE(result[0] == preset_entries[2]);
    EXPECT_TRUE(result[1] == preset_entries[3]);
    EXPECT_TRUE(result[2] == preset_entries[9]);
    EXPECT_TRUE(result[3] == preset_entries[5]);
    EXPECT_TRUE(result[4] == preset_entries[7]);
  }

  {
    // soft soft
    auto preset_entries = create_preset_entries();
    TimeWindowLookup time_window_lookup;
    for (auto & entry : preset_entries) {
      time_window_lookup.add_entry(entry);
    }

    auto result = time_window_lookup.lookup(
      TimeWindow{
      Time::from_localtime("Jan 01 00:01:00 2023"),
      Time::from_localtime("Jan 01 00:04:00 2023")
    },
      true,
      true
    );

    // for (auto & entry : result) {
    //   std::cout << entry.id << ": "
    //     << entry.time_window.start.to_localtime() << " - "
    //     << entry.time_window.end.to_localtime() << std::endl;
    // }

    // Check result
    ASSERT_EQ(result.size(), 9lu);

    EXPECT_TRUE(result[0] == preset_entries[1]);
    EXPECT_TRUE(result[1] == preset_entries[6]);
    EXPECT_TRUE(result[2] == preset_entries[0]);
    EXPECT_TRUE(result[3] == preset_entries[2]);
    EXPECT_TRUE(result[4] == preset_entries[9]);
    EXPECT_TRUE(result[5] == preset_entries[3]);
    EXPECT_TRUE(result[6] == preset_entries[4]);
    EXPECT_TRUE(result[7] == preset_entries[5]);
    EXPECT_TRUE(result[8] == preset_entries[7]);
  }
}

TEST_F(TestTimeWindowLookup, get_time_window) {
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using TimeWindowLookup = rmf2_scheduler::TimeWindowLookup;  // NOLINT(build/namespaces)

  {
    // Empty lookup
    TimeWindowLookup time_window_lookup;

    EXPECT_THROW_EQ(
      auto time_window = time_window_lookup.get_time_window(),
      std::out_of_range("No entries in TimeWindowLookup, cannot retrieve time_window!")
    );
  }

  auto preset_entries = create_preset_entries();
  TimeWindowLookup time_window_lookup;
  for (auto & entry : preset_entries) {
    time_window_lookup.add_entry(entry);
  }

  {
    // time window based on start time
    auto time_window = time_window_lookup.get_time_window();

    EXPECT_EQ(
      time_window.start,
      Time::from_localtime("Jan 01 00:00:00 2023")
    );
    EXPECT_EQ(
      time_window.end,
      Time::from_localtime("Jan 01 00:05:00 2023")
    );
  }

  {
    // time window based on start time
    auto time_window = time_window_lookup.get_time_window(false);

    EXPECT_EQ(
      time_window.start,
      Time::from_localtime("Jan 01 00:00:00 2023")
    );
    EXPECT_EQ(
      time_window.end,
      Time::from_localtime("Jan 01 00:06:00 2023")
    );
  }
}
