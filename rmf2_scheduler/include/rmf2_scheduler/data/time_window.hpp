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

#ifndef RMF2_SCHEDULER__DATA__TIME_WINDOW_HPP_
#define RMF2_SCHEDULER__DATA__TIME_WINDOW_HPP_

#include "rmf2_scheduler/data/time.hpp"

namespace rmf2_scheduler
{

namespace data
{

struct TimeWindow
{
  data::Time start;
  data::Time end;

  inline bool operator==(const TimeWindow & rhs) const
  {
    if (this->start != rhs.start) {return false;}
    if (this->end != rhs.end) {return false;}
    return true;
  }

  inline bool operator!=(const TimeWindow & rhs) const
  {
    return !(*this == rhs);
  }
};

}  // namespace data
}  // namespace rmf2_scheduler


#endif  // RMF2_SCHEDULER__DATA__TIME_WINDOW_HPP_
