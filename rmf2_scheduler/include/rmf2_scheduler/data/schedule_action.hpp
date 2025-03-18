// Copyright 2025 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__DATA__SCHEDULE_ACTION_HPP_
#define RMF2_SCHEDULER__DATA__SCHEDULE_ACTION_HPP_

#include <memory>
#include <optional>
#include <string>

#include "rmf2_scheduler/data/event.hpp"
#include "rmf2_scheduler/data/task.hpp"
#include "rmf2_scheduler/data/process.hpp"
#include "rmf2_scheduler/utils/utils.hpp"

namespace rmf2_scheduler
{
namespace data
{

namespace action_type
{

// Event Operation
extern const char EVENT_PREFIX[];
extern const char EVENT_ADD[];
extern const char EVENT_UPDATE[];
extern const char EVENT_DELETE[];

// Task Operation
extern const char TASK_PREFIX[];
extern const char TASK_ADD[];
extern const char TASK_UPDATE[];
extern const char TASK_DELETE[];

// Process Operation
extern const char PROCESS_PREFIX[];
extern const char PROCESS_ADD[];
extern const char PROCESS_ATTACH_NODE[];
extern const char PROCESS_ADD_DEPENDENCY[];
extern const char PROCESS_UPDATE[];
extern const char PROCESS_UPDATE_START_TIME[];
extern const char PROCESS_DETACH_NODE[];
extern const char PROCESS_DELETE[];
extern const char PROCESS_DELETE_DEPENDENCY[];
extern const char PROCESS_DELETE_ALL[];

// Series Operation
extern const char SERIES_PREFIX[];
extern const char SERIES_ADD[];
extern const char SERIES_UPDATE[];
extern const char SERIES_EXPAND_UNTIL[];
extern const char SERIES_UPDATE_CRON[];
extern const char SERIES_UPDATE_UNTIL[];
extern const char SERIES_UPDATE_OCCURRENCE[];
extern const char SERIES_DELETE_OCCURRENCE[];
extern const char SERIES_DELETE[];
extern const char SERIES_DELETE_ALL[];

}  // namespace action_type

/// Schedule Action
struct ScheduleAction
{
  std::string type;

  // Common
  std::optional<std::string> id;

  // Event specific
  std::shared_ptr<Event> event;

  // Task specific
  std::shared_ptr<Task> task;

  // Process specific
  std::shared_ptr<Process> process;
  std::optional<std::string> node_id;
  std::optional<std::string> source_id;
  std::optional<std::string> destination_id;
  std::optional<std::string> edge_type;

  inline bool operator==(const ScheduleAction & rhs) const
  {
    if (this->type != rhs.type) {return false;}
    if (this->id != rhs.id) {return false;}
    if (!utils::is_deep_equal(this->event, rhs.event)) {return false;}
    if (!utils::is_deep_equal(this->task, rhs.task)) {return false;}
    if (!utils::is_deep_equal(this->process, rhs.process)) {return false;}
    if (this->node_id != rhs.node_id) {return false;}
    if (this->source_id != rhs.source_id) {return false;}
    if (this->destination_id != rhs.destination_id) {return false;}
    if (this->edge_type != rhs.edge_type) {return false;}
    return true;
  }

  inline bool operator!=(const ScheduleAction & rhs) const
  {
    return !(*this == rhs);
  }
};

}  // namespace data
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__DATA__SCHEDULE_ACTION_HPP_
