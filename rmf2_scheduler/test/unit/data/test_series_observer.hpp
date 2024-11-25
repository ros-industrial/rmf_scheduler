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

#ifndef UNIT__DATA__TEST_SERIES_OBSERVER_HPP_
#define UNIT__DATA__TEST_SERIES_OBSERVER_HPP_

#include <vector>

#include "rmf2_scheduler/data/series.hpp"

namespace rmf2_scheduler
{
namespace data
{

class TestSeriesObserver : public Series::ObserverBase
{
public:
  TestSeriesObserver() = default;
  virtual ~TestSeriesObserver() {}

  struct Record
  {
    Time record_time;
  };

  struct AddOccurrenceRecord : public Record
  {
    Occurrence ref_occurrence;
    Occurrence new_occurrence;
  };

  struct UpdateOccurrenceRecord : public Record
  {
    Occurrence old_occurrence;
    Time new_time;
  };

  struct DeleteOccurrenceRecord : public Record
  {
    Occurrence occurrence;
  };

  void on_add_occurrence(
    const Occurrence & ref_occurrence,
    const Occurrence & new_occurrence
  ) override
  {
    add_occurrence_record_.push_back(
      {{_now()}, ref_occurrence, new_occurrence}
    );
  }

  void on_update_occurrence(
    const Occurrence & old_occurrence,
    const Time & new_time
  ) override
  {
    update_occurrence_record_.push_back(
      {{_now()}, old_occurrence, new_time}
    );
  }

  void on_delete_occurrence(
    const Occurrence & old_occurrence
  ) override
  {
    delete_occurrence_record_.push_back(
      {{_now()}, old_occurrence}
    );
  }

  const std::vector<AddOccurrenceRecord> &
  get_add_occurrence_record() const
  {
    return add_occurrence_record_;
  }

  const std::vector<UpdateOccurrenceRecord> &
  get_update_occurrence_record() const
  {
    return update_occurrence_record_;
  }

  const std::vector<DeleteOccurrenceRecord> &
  get_delete_occurrence_record() const
  {
    return delete_occurrence_record_;
  }

  Time _now() const
  {
    return Time(std::chrono::system_clock::now());
  }

private:
  std::vector<AddOccurrenceRecord> add_occurrence_record_;
  std::vector<UpdateOccurrenceRecord> update_occurrence_record_;
  std::vector<DeleteOccurrenceRecord> delete_occurrence_record_;
};

}  // namespace data
}  // namespace rmf2_scheduler

#endif  // UNIT__DATA__TEST_SERIES_OBSERVER_HPP_
