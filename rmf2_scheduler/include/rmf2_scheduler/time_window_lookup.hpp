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

#ifndef RMF2_SCHEDULER__TIME_WINDOW_LOOKUP_HPP_
#define RMF2_SCHEDULER__TIME_WINDOW_LOOKUP_HPP_

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "rmf2_scheduler/data/time.hpp"
#include "rmf2_scheduler/data/time_window.hpp"

namespace rmf2_scheduler
{

class TimeWindowLookup
{
public:
  struct Entry
  {
    std::string id;
    std::string type;
    data::TimeWindow time_window;

    inline bool operator==(const Entry & rhs) const
    {
      if (this->id != rhs.id) {return false;}
      if (this->type != rhs.type) {return false;}
      if (this->time_window != rhs.time_window) {return false;}
      return true;
    }

    inline bool operator!=(const Entry & rhs) const
    {
      return !(*this == rhs);
    }
  };

  /// Add an entry to the lookup
  void add_entry(
    const Entry & entry
  );

  /// Remove an entry from the lookup
  void remove_entry(
    const std::string & id
  );

  /// Retrieve an entry from the lookup
  Entry get_entry(
    const std::string & id
  ) const;

  /// Lookup entries
  std::vector<Entry> lookup(
    const data::TimeWindow & time_window,
    bool soft_lower_bound = false,
    bool soft_upper_bound = false
  ) const;

  bool empty() const;

  data::TimeWindow get_time_window(
    bool soft_upper_bound = true
  ) const;

private:
  std::multimap<rs_time_point_value_t, Entry> start_lookup_;
  std::multimap<rs_time_point_value_t, Entry> end_lookup_;
  std::unordered_map<std::string, Entry> id_lookup_;
};

}  // namespace rmf2_scheduler


#endif  // RMF2_SCHEDULER__TIME_WINDOW_LOOKUP_HPP_
