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

const EventsHandler::StartTimeLookup & EventsHandler::get_lookup() const
{
  return start_time_lookup_;
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

std::vector<Event> EventsHandler::lookup_next_events(
  uint64_t start_time) const
{
  StartTimeLookup::const_iterator itr, itr_low;
  itr_low = start_time_lookup_.lower_bound(start_time);

  std::vector<Event> looked_up_events;
  for (itr = itr_low; itr != start_time_lookup_.cend(); itr++) {
    looked_up_events.push_back(event_list_.at(itr->second));
  }
  return looked_up_events;
}

std::vector<Event> EventsHandler::lookup_earliest_events() const
{
  if (event_list_.empty()) {
    return {};
  }

  std::vector<Event> looked_up_events;
  auto itr = start_time_lookup_.cbegin();
  auto range = start_time_lookup_.equal_range(itr->first);
  for (auto i = range.first; i != range.second; ++i) {
    looked_up_events.push_back(event_list_.at(i->second));
  }
  return looked_up_events;
}

void EventsHandler::add_event(const Event & event)
{
  if (has_event(event.id)) {
    throw EventsHandlerIDException(
            "add_event error."
            "Event ID [%s] already exist",
            event.id.c_str());
  }
  event_list_[event.id] = event;
  add_lookup(event);
}

void EventsHandler::update_event(
  const Event & event)
{
  if (!has_event(event.id)) {
    throw EventsHandlerIDException(
            "update_event error. "
            "Event ID [%s] doesn't exist",
            event.id.c_str());
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

void EventsHandler::delete_event(const std::string & id)
{
  const auto & event = event_list_.at(id);
  delete_lookup(event);
  event_list_.erase(id);
}

void EventsHandler::delete_events(
  uint64_t start_time,
  uint64_t end_time)
{
  StartTimeLookup::const_iterator itr_low, itr_up;
  itr_low = start_time_lookup_.lower_bound(start_time);
  itr_up = start_time_lookup_.upper_bound(end_time);

  for (auto itr = itr_low; itr != itr_up; itr++) {
    event_list_.erase(itr->second);
    start_time_lookup_.erase(itr);
  }
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
