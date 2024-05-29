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

#include "rmf_scheduler/data/event.hpp"

namespace rmf_scheduler
{

namespace data
{

Event::Event(
  std::string _description,
  std::string _type,
  uint64_t _start_time,
  uint64_t _duration,
  std::string _id,
  std::string _series_id,
  std::string _resource_id,
  std::string _dependency_id,
  std::string _event_details)
: description(_description),
  type(_type),
  start_time(_start_time),
  duration(_duration),
  id(_id),
  series_id(_series_id),
  resource_id(_resource_id),
  dependency_id(_dependency_id),
  event_details(_event_details)
{
}

}  // namespace data

}  // namespace rmf_scheduler
