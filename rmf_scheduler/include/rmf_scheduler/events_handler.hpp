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

#ifndef RMF_SCHEDULER__EVENTS_HANDLER_HPP_
#define RMF_SCHEDULER__EVENTS_HANDLER_HPP_

#include <cstdarg>

#include <exception>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "event.hpp"

namespace rmf_scheduler
{

class EventsHandler
{
public:
  using EventList = std::unordered_map<std::string, Event>;
  using StartTimeLookup = std::multimap<uint64_t, std::string>;

  bool has_event(const std::string & id) const;

  const Event & get_event(const std::string & id) const;

  const EventList & get_all_events() const;

  const StartTimeLookup & get_lookup() const;

  std::vector<Event> lookup_events(
    uint64_t start_time,
    uint64_t end_time) const;

  std::vector<Event> lookup_next_events(
    uint64_t start_time) const;

  std::vector<Event> lookup_earliest_events() const;

  void add_event(const Event &);

  void update_event(const Event &);

  void delete_event(const std::string & id);

  void delete_events(
    uint64_t start_time,
    uint64_t end_time);

protected:
  void add_lookup(const Event & event);

  void delete_lookup(const Event & event);

private:
  EventList event_list_;
  StartTimeLookup start_time_lookup_;
};

class EventsHandlerIDException : public std::exception
{
public:
  EventsHandlerIDException(const char * msg, ...)
  {
    buffer_ = new char[4096];
    va_list args;
    va_start(args, msg);
    vsprintf(buffer_, msg, args);
    va_end(args);
  }

  ~EventsHandlerIDException()
  {
    delete buffer_;
  }

  const char * what() const noexcept
  {
    return buffer_;
  }

private:
  char * buffer_;
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__EVENTS_HANDLER_HPP_
