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

#include "rmf_scheduler/events_handler.hpp"

namespace rmf_scheduler
{

bool EventsHandler::has_event(const std::string & id) const
{
  return event_list_.find(id) != event_list_.end();
}

const Event & EventsHandler::get_event(const std::string & id) const
{
  return event_list_.at(id);
}

const EventsHandler::EventList & EventsHandler::get_all_events() const
{
  return event_list_;
}

std::vector<Event> EventsHandler::lookup_events(
  uint64_t start_time,
  uint64_t end_time) const
{
  StartTimeLookup::const_iterator itr, itr_low, itr_up;
  itr_low = start_time_lookup_.lower_bound(start_time);
  itr_up = start_time_lookup_.upper_bound(end_time);

  std::vector<Event> looked_up_events;
  for (itr = itr_low; itr != itr_up; itr++) {
    looked_up_events.push_back(event_list_.at(itr->second));
  }
  return looked_up_events;
}

void EventsHandler::add_event(const Event & event)
{
  if (has_event(event.id)) {
    throw EventsHandlerIDException(
            event.id,
            "add_event error."
            "Event ID already exist");
  }
  event_list_[event.id] = event;
  add_lookup(event);
}

void EventsHandler::update_event(
  const Event & event)
{
  if (!has_event(event.id)) {
    throw EventsHandlerIDException(
            event.id,
            "update_event error. "
            "Event ID doesn't exist");
  }

  // Update lookup table if task start time is changed
  auto old_event = get_event(event.id);
  if (old_event.start_time != event.start_time) {
    delete_lookup(old_event);
    add_lookup(event);
  }

  // Update event in the list
  event_list_[event.id] = event;
}

void EventsHandler::add_lookup(const Event & event)
{
  start_time_lookup_.emplace(event.start_time, event.id);
}

void EventsHandler::delete_lookup(const Event & event)
{
  auto range = start_time_lookup_.equal_range(event.start_time);
  for (auto itr = range.first; itr != range.second; itr++) {
    if (itr->second == event.id) {
      start_time_lookup_.erase(itr);
      break;
    }
  }
}

}  // namespace rmf_scheduler
