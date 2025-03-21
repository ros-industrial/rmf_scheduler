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

#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include "rmf2_scheduler_py/py_utils.hpp"
#include "rmf2_scheduler_py/data/schedule_action.hpp"
#include "rmf2_scheduler/data/schedule_action.hpp"
#include "pybind11_json/pybind11_json.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_schedule_action_py(py::module & m)
{
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  py::module m_data = m.def_submodule("data");

  // action type
  py::module m_action_type = m_data.def_submodule("action_type");

  py_utils::def_str_const(
    m_action_type,
    "EVENT_ADD",
    action_type::EVENT_ADD
  );
  py_utils::def_str_const(
    m_action_type,
    "EVENT_UPDATE",
    action_type::EVENT_UPDATE
  );
  py_utils::def_str_const(
    m_action_type,
    "EVENT_DELETE",
    action_type::EVENT_DELETE
  );

  py_utils::def_str_const(
    m_action_type,
    "TASK_ADD",
    action_type::TASK_ADD
  );
  py_utils::def_str_const(
    m_action_type,
    "TASK_UPDATE",
    action_type::TASK_UPDATE
  );
  py_utils::def_str_const(
    m_action_type,
    "TASK_DELETE",
    action_type::TASK_DELETE
  );

  py_utils::def_str_const(
    m_action_type,
    "PROCESS_ADD",
    action_type::PROCESS_ADD
  );
  py_utils::def_str_const(
    m_action_type,
    "PROCESS_ATTACH_NODE",
    action_type::PROCESS_ATTACH_NODE
  );
  py_utils::def_str_const(
    m_action_type,
    "PROCESS_ADD_DEPENDENCY",
    action_type::PROCESS_ADD_DEPENDENCY
  );
  py_utils::def_str_const(
    m_action_type,
    "PROCESS_UPDATE",
    action_type::PROCESS_UPDATE
  );
  py_utils::def_str_const(
    m_action_type,
    "PROCESS_UPDATE_START_TIME",
    action_type::PROCESS_UPDATE_START_TIME
  );
  py_utils::def_str_const(
    m_action_type,
    "PROCESS_DETACH_NODE",
    action_type::PROCESS_DETACH_NODE
  );
  py_utils::def_str_const(
    m_action_type,
    "PROCESS_DELETE",
    action_type::PROCESS_DELETE
  );
  py_utils::def_str_const(
    m_action_type,
    "PROCESS_DELETE_DEPENDENCY",
    action_type::PROCESS_DELETE_DEPENDENCY
  );
  py_utils::def_str_const(
    m_action_type,
    "PROCESS_DELETE_ALL",
    action_type::PROCESS_DELETE_ALL
  );

  // ScheduleAction
  py::class_<ScheduleAction>(
    m_data,
    "ScheduleAction",
    R"(
    Change Action to the schedule
    )"
  )
  .def(py::init<>())
  .def_readwrite(
    "type",
    &ScheduleAction::type
  )
  .def_readwrite(
    "id",
    &ScheduleAction::id
  )
  .def_readwrite(
    "event",
    &ScheduleAction::event
  )
  .def_readwrite(
    "task",
    &ScheduleAction::task
  )
  .def_readwrite(
    "process",
    &ScheduleAction::process
  )
  .def_readwrite(
    "node_id",
    &ScheduleAction::node_id
  )
  .def_readwrite(
    "source_id",
    &ScheduleAction::source_id
  )
  .def_readwrite(
    "destination_id",
    &ScheduleAction::destination_id
  )
  .def_readwrite(
    "edge_type",
    &ScheduleAction::edge_type
  )
  .def(py::self == py::self)
  .def(py::self != py::self)

  // Serialization
  .def(
    "json",
    [](
      ScheduleAction & self
    )
    {
      return nlohmann::json(self);
    }
  )
  .def_static(
    "from_json",
    [](
      const nlohmann::json & j
    )
    {
      return j.template get<ScheduleAction>();
    }
  )
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
