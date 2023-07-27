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

#include "boost/uuid/uuid.hpp"
#include "boost/uuid/random_generator.hpp"
#include "boost/uuid/uuid_io.hpp"

#include "rmf_scheduler/event.hpp"
#include "rmf_scheduler/conflict_resolver/identifier.hpp"
#include "rmf_scheduler/conflict_resolver/cp_resolver.hpp"


using EventChange = rmf_scheduler::conflict_resolver::EventChange;
using CpResolver = rmf_scheduler::conflict_resolver::CpResolver;
using Event = rmf_scheduler::Event;
using Identifier = rmf_scheduler::conflict_resolver::Identifier;

std::string gen_uuid()
{
  boost::uuids::uuid uuid = boost::uuids::random_generator()();
  return boost::uuids::to_string(uuid);
}

std::unordered_map<std::string, Event> load_robot_task_events()
{
  std::unordered_map<std::string, Event> events;
  const std::vector<std::string> locations = {
    "location_1",
    "location_2",
    "location_3",
    "location_4",
    "location_5",
    "area_54"
  };
  const uint64_t start = 0;
  const uint64_t duration = 60;
  for (int i = 0; i < 5; i++) {  // 10 robots
    const std::string robot = "robot_" + std::to_string(i);
    for (int j = 0; j < 10; j++) {  // 10 events
      std::string id = gen_uuid();
      events.emplace(
        std::make_pair(
          id,
          Event{
        "",                                 // description
        "default/robot_task",               // type
        start + duration * j,               // start time
        duration,                           // duration
        id,                                 // id
        "",                                 // series id
        "",                                 // dag id
        false,                              // flight schedule
        robot,                              // robot
        locations[j % locations.size()]     // random location so no clashes
      }
        )
      );
    }
  }
  return events;
}

std::unordered_map<std::string, Event> load_flight_events()
{
  std::unordered_map<std::string, Event> events;
  const uint64_t start = 0;
  const uint64_t duration = 60;
  const std::vector<std::string> locations = {
    "location_1",
    "location_2",
    "location_3",
    "location_4",
    "location_5",
    "area_54"
  };
  for (int i = 0; i < 10; i++) {  // 10 events
    std::string id = gen_uuid();
    events.emplace(
      std::make_pair(
        id,
        Event{
      "",                                 // description
      "flight-schedule",                  // type
      start + 60 * i,                     // start time
      duration,                           // duration
      id,                                 // id
      "",                                 // series id
      "",                                 // dag id
      true,                               // flight schedule
      "",                                 // robot
      locations[i % locations.size()]     // random location so no clashes
    }
      )
    );
  }
  return events;
}


std::unordered_map<std::string, Event> load_no_clashing_events()
{
  std::unordered_map<std::string, Event> events;
  const uint64_t start = 0;
  const uint64_t duration = 60;
  for (int i = 0; i < 2; i++) {  // 10 robots
    const std::string robot = "robot_" + std::to_string(i);
    for (int j = 0; j < 3; j++) {  // 10 events
      std::string id = gen_uuid();
      events.emplace(
        std::make_pair(
          id,
          Event{
        "",                         // description
        "default/robot_task",       // type
        start + duration * j,       // start time
        duration,                   // duration
        id,                         // id
        "",                         // series id
        "",                         // dag id
        false,                      // flight schedule
        robot,                      // robot
        gen_uuid()                  // random location so no clashes
      }
        )
      );
    }
  }
  return events;
}

void print_event_map(const std::unordered_map<std::string, Event> & events)
{
  for (auto event : events) {
    std::cout << "Event \n"
              << std::endl
              << "\tid: "
              << event.second.id
              << "\n \tstart time: "
              << event.second.start_time
              << "\n \tduration: "
              << event.second.duration
              << "\n \tis flight schedule: "
              << event.second.is_flight_schedule
              << "\n \trobot: "
              << event.second.robot
              << "\n \tlocation: "
              << event.second.location
              << std::endl;
  }
}

auto categorise_by_robots(const std::unordered_map<std::string, Event> & events)
-> std::unordered_map<std::string, std::unordered_map<std::string, Event>>
{
  std::unordered_map<std::string, std::unordered_map<std::string, Event>> categorised_events;
  for (auto event : events) {
    if (event.second.is_flight_schedule) {
      // this event is not part of any robot's tasks
      continue;
    }
    categorised_events[event.second.robot][event.second.id] = event.second;
  }
  return categorised_events;
}

auto categorise_by_location(const std::unordered_map<std::string, Event> & events)
-> std::unordered_map<std::string, std::unordered_map<std::string, Event>>
{
  std::unordered_map<std::string, std::unordered_map<std::string, Event>> categorised_events;
  for (auto event : events) {
    categorised_events[event.second.location][event.second.id] = event.second;
  }
  return categorised_events;
}

int main()
{
  const uint64_t WINDOW_START = 0;
  const uint64_t WINDOW_END = 60 * 24;  // 24 hours

  // Create conflict identifier
  Identifier conflict_identifier = Identifier();
  std::cout << "Created identifier." << std::endl;

  // Create model builder
  std::shared_ptr<CpResolver> cp_resolver = CpResolver::make();
  std::cout << "Created resolver." << std::endl;

  // PART ONE: //////////////////////////////////////////////////////////////////////////
  // Check no unnecessary rescheduling
  std::unordered_map<std::string, Event> no_clashing_events;
  no_clashing_events = load_no_clashing_events();
  auto robot_categorised_event_maps =
    categorise_by_robots(no_clashing_events);
  auto location_categorised_event_maps =
    categorise_by_location(no_clashing_events);
  print_event_map(no_clashing_events);
  std::cout << "Loaded no clashing events." << std::endl;

  // Ensure that there are no clashing events throught the identifier
  std::vector<std::pair<std::string, std::string>> check_clashing_events;

  // Check if locations do not clash
  for (auto location : location_categorised_event_maps) {
    check_clashing_events = conflict_identifier.identify_conflicts(location.second);
    std::cout << "Checked if events clash based on locattion: {" << location.first << "}" <<
      std::endl;
    if (check_clashing_events.size() != 0) {
      std::cout << "Apparently the 'no conflict' schedule has conflicts for location {"
                << location.first
                << "}!"
                << std::endl;
    }
    check_clashing_events.clear();
    std::cout << "Checked conflicts for non clashing schedule." << std::endl;
  }
  // clear map
  check_clashing_events.clear();

  // Check if robots do not clash
  for (auto robot : robot_categorised_event_maps) {
    check_clashing_events = conflict_identifier.identify_conflicts(robot.second);
    std::cout << "Checked if events clash based on robot: {" << robot.first << "}" << std::endl;
    if (check_clashing_events.size() != 0) {
      std::cout << "Apparently the 'no conflict' schedule has conflicts for robot {"
                << robot.first
                << "}!"
                << std::endl;
    }
    check_clashing_events.clear();
    std::cout << "Checked conflicts for non clashing schedule." << std::endl;
  }
  // clear map
  check_clashing_events.clear();

  // Initalize vector of event changes
  std::unordered_map<std::string, EventChange> no_clash_event_changes;

  // Get deconflited schedule changes
  no_clash_event_changes = cp_resolver->deconflict(
    no_clashing_events, WINDOW_START, WINDOW_END,
    true);
  std::cout << "Deconflicted non clashing schedules." << std::endl;

  // Show make sure that there are no changes
  std::cout << "TEST NO CLASHING EVENTS" << std::endl;
  std::cout << "Events size: " << no_clash_event_changes.size() << std::endl;
  if (no_clash_event_changes.size() != 0) {
    std::cout << "TEST NO CLASHING FAILED" << std::endl;
  } else {
    std::cout << "TEST NO CLASHING OK" << std::endl;
  }

  // clear resolver
  cp_resolver->clear();
  std::cout << "Cleared the cp resolver model builder." << std::endl;

  // Clear categoriesed event maps
  robot_categorised_event_maps.clear();
  location_categorised_event_maps.clear();
  std::cout << "Cleared the categorised event maps" << std::endl;


  // PART TWO: //////////////////////////////////////////////////////////////////////////
  // Check that deconfliction is done properly

  // Load tasks
  std::cout << "creating robot task events map" << std::endl;
  std::unordered_map<std::string, Event> robot_task_events;
  robot_task_events = load_robot_task_events();
  std::cout << "Loaded robot tasks." << std::endl;

  // Load flight schedule based events
  std::unordered_map<std::string, Event> flight_schedule_events;
  flight_schedule_events = load_flight_events();
  std::cout << "Loaded flight schedule." << std::endl;

  // Combine the two types of events into a single map
  std::unordered_map<std::string, EventChange> clashing_event_changes;
  std::unordered_map<std::string, Event> combined_schedule;
  for (auto itr : robot_task_events) {
    combined_schedule.emplace(itr);
  }
  for (auto itr : flight_schedule_events) {
    combined_schedule.emplace(itr);
  }
  std::cout << "Combined schedule." << std::endl;
  print_event_map(combined_schedule);

  // Categorise events
  robot_categorised_event_maps = categorise_by_robots(combined_schedule);
  location_categorised_event_maps = categorise_by_location(combined_schedule);


  // Show current conflicts
  check_clashing_events.clear();
  std::cout << "Identifying clashes based on robots." << std::endl;
  for (auto robot : robot_categorised_event_maps) {
    std::cout << "Identifying clashes for robot {" << robot.first << "}" << std::endl;
    check_clashing_events = conflict_identifier.identify_conflicts(robot.second);
    for (auto pair : check_clashing_events) {
      auto first_event_itr = combined_schedule.find(pair.first);
      auto second_event_itr = combined_schedule.find(pair.second);
      if (first_event_itr == combined_schedule.end() ||
        second_event_itr == combined_schedule.end())
      {
        std::cout << "Events are non existant" << std::endl;
      }
      std::cout << "Event {"
                << pair.first
                << "} of start time {"
                << first_event_itr->second.start_time
                << "} and duration {"
                << first_event_itr->second.start_time
                << "}"
                << std::endl
                << "Clashes with,"
                << std::endl
                << "Event {"
                << pair.second
                << "} of start time {"
                << second_event_itr->second.start_time
                << "} and duration {"
                << second_event_itr->second.start_time
                << "}"
                << std::endl
                << std::endl;
    }
    check_clashing_events.clear();
  }

  std::cout << "Identifying clashes based on location." << std::endl;
  for (auto location : location_categorised_event_maps) {
    std::cout << "Identifying clashes for location {" << location.first << "}" << std::endl;
    check_clashing_events = conflict_identifier.identify_conflicts(location.second);
    for (auto pair : check_clashing_events) {
      auto first_event_itr = combined_schedule.find(pair.first);
      auto second_event_itr = combined_schedule.find(pair.second);
      if (first_event_itr == combined_schedule.end() ||
        second_event_itr == combined_schedule.end())
      {
        std::cout << "Events are non existant" << std::endl;
      }
      std::cout << "Event {"
                << pair.first
                << "} of start time {"
                << first_event_itr->second.start_time
                << "} and duration {"
                << first_event_itr->second.start_time
                << "}"
                << std::endl
                << "Clashes with,"
                << std::endl
                << "Event {"
                << pair.second
                << "} of start time {"
                << second_event_itr->second.start_time
                << "} and duration {"
                << second_event_itr->second.start_time
                << "}"
                << std::endl
                << std::endl;
    }
    check_clashing_events.clear();
  }

  // Solve clashing schedules
  clashing_event_changes =
    cp_resolver->deconflict(combined_schedule, WINDOW_START, WINDOW_END, true);

  // Show changes and update rescheduled events
  std::cout << "Event changes " << std::endl;
  for (auto change : clashing_event_changes) {
    std::cout << "Event: "
              << change.first
              << std::endl
              << "changed from: "
              << change.second.original_start_time
              << "to: "
              << change.second.final_start_time
              << std::endl;
    auto changed_event_itr = combined_schedule.find(change.second.id);
    if (changed_event_itr == combined_schedule.end()) {
      std::cout << "Event: " << change.second.id << " not found!" << std::endl;
      return 1;
    }
    // Change event time
    changed_event_itr->second.start_time = change.second.final_start_time;
  }

  // Check events
  check_clashing_events.clear();
  check_clashing_events = conflict_identifier.identify_conflicts(combined_schedule);

  // Check if there are any conflicts. There shouldn't have any
  if (check_clashing_events.size() != 0) {
    std::cout << "The conflict resolver didn't really do its job init?" << std::endl;
  } else {
    std::cout << "Conflicts successfully resolved" << std::endl;
  }
}
