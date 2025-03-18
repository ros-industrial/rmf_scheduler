// Copyright 2024-2025 ROS Industrial Consortium Asia Pacific
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

#include <pybind11/pybind11.h>

#include "rmf2_scheduler_py/cache/action.hpp"
#include "rmf2_scheduler_py/cache/event_action.hpp"
#include "rmf2_scheduler_py/cache/process_action.hpp"
#include "rmf2_scheduler_py/cache/schedule_cache.hpp"
#include "rmf2_scheduler_py/cache/task_action.hpp"
#include "rmf2_scheduler_py/data/edge.hpp"
#include "rmf2_scheduler_py/data/event.hpp"
#include "rmf2_scheduler_py/data/duration.hpp"
#include "rmf2_scheduler_py/data/graph.hpp"
#include "rmf2_scheduler_py/data/json_serializer.hpp"
#include "rmf2_scheduler_py/data/node.hpp"
#include "rmf2_scheduler_py/data/process.hpp"
#include "rmf2_scheduler_py/data/schedule_action.hpp"
#include "rmf2_scheduler_py/data/schedule_change_record.hpp"
#include "rmf2_scheduler_py/data/task.hpp"
#include "rmf2_scheduler_py/data/time.hpp"
#include "rmf2_scheduler_py/data/time_window.hpp"
#include "rmf2_scheduler_py/scheduler.hpp"
#include "rmf2_scheduler_py/scheduler_options.hpp"
#include "rmf2_scheduler_py/storage/schedule_stream.hpp"
#include "rmf2_scheduler_py/system_time_executor.hpp"

PYBIND11_MODULE(core, m)
{
  // DATA
  rmf2_scheduler_py::data::init_duration_py(m);
  rmf2_scheduler_py::data::init_time_py(m);
  rmf2_scheduler_py::data::init_time_window_py(m);
  rmf2_scheduler_py::data::init_event_py(m);
  rmf2_scheduler_py::data::init_task_py(m);
  rmf2_scheduler_py::data::init_edge_py(m);
  rmf2_scheduler_py::data::init_node_py(m);
  rmf2_scheduler_py::data::init_graph_py(m);
  rmf2_scheduler_py::data::init_process_py(m);
  rmf2_scheduler_py::data::init_schedule_action_py(m);
  rmf2_scheduler_py::data::init_schedule_change_record_py(m);
  rmf2_scheduler_py::data::init_json_serializer_py(m);

  // CACHE
  rmf2_scheduler_py::cache::init_schedule_cache_py(m);
  rmf2_scheduler_py::cache::init_action_py(m);
  rmf2_scheduler_py::cache::init_event_action_py(m);
  rmf2_scheduler_py::cache::init_task_action_py(m);
  rmf2_scheduler_py::cache::init_process_action_py(m);

  // STORAGE
  rmf2_scheduler_py::storage::init_schedule_stream_py(m);

  // ROOT
  rmf2_scheduler_py::init_scheduler_options_py(m);
  rmf2_scheduler_py::init_system_time_executor_py(m);
  rmf2_scheduler_py::init_scheduler_py(m);
}
