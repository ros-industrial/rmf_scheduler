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

#include "rmf2_scheduler/data/schedule_action.hpp"

namespace rmf2_scheduler
{
namespace data
{
namespace action_type
{
// Event Operation
const char EVENT_PREFIX[] = "EVENT_";
const char EVENT_ADD[] = "EVENT_ADD";
const char EVENT_UPDATE[] = "EVENT_UPDATE";
const char EVENT_DELETE[] = "EVENT_DELETE";

// Task Operation
const char TASK_PREFIX[] = "TASK_";
const char TASK_ADD[] = "TASK_ADD";
const char TASK_UPDATE[] = "TASK_UPDATE";
const char TASK_DELETE[] = "TASK_DELETE";

// Process Operation
const char PROCESS_PREFIX[] = "PROCESS_";
const char PROCESS_ADD[] = "PROCESS_ADD";
const char PROCESS_ATTACH_NODE[] = "PROCESS_ATTACH_NODE";
const char PROCESS_ADD_DEPENDENCY[] = "PROCESS_ADD_DEPENDENCY";
const char PROCESS_UPDATE[] = "PROCESS_UPDATE";
const char PROCESS_UPDATE_START_TIME[] = "PROCESS_UPDATE_START_TIME";
const char PROCESS_DETACH_NODE[] = "PROCESS_DETACH_NODE";
const char PROCESS_DELETE[] = "PROCESS_DELETE";
const char PROCESS_DELETE_DEPENDENCY[] = "PROCESS_DELETE_DEPENDENCY";
const char PROCESS_DELETE_ALL[] = "PROCESS_DELETE_ALL";

// Series Operation
const char SERIES_PREFIX[] = "SERIES_";
const char SERIES_ADD[] = "SERIES_ADD";
const char SERIES_UPDATE[] = "SERIES_UPDATE";
const char SERIES_EXPAND_UNTIL[] = "SERIES_EXPAND_UNTIL";
const char SERIES_UPDATE_CRON[] = "SERIES_UPDATE_CRON";
const char SERIES_UPDATE_UNTIL[] = "SERIES_UPDATE_UNTIL";
const char SERIES_UPDATE_OCCURRENCE[] = "SERIES_UPDATE_OCCURRENCE";
const char SERIES_DELETE_OCCURRENCE[] = "SERIES_DELETE_OCCURRENCE";
const char SERIES_DELETE[] = "SERIES_DELETE";
const char SERIES_DELETE_ALL[] = "SERIES_DELETE_ALL";

}  // namespace action_type
}  // namespace data
}  // namespace rmf2_scheduler
