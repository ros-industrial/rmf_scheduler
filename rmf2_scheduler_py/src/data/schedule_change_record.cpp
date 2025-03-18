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

#include <pybind11/stl.h>
#include <pybind11/operators.h>

#include "rmf2_scheduler_py/py_utils.hpp"
#include "rmf2_scheduler_py/data/schedule_change_record.hpp"
#include "rmf2_scheduler/data/schedule_change_record.hpp"

namespace rmf2_scheduler_py
{

namespace data
{

void init_schedule_change_record_py(py::module & m)
{
  py::module m_data = m.def_submodule("data");

  // record_data_type
  py::module m_record_data_type = m_data.def_submodule("record_data_type");
  py_utils::def_str_const(
    m_record_data_type,
    "EVENT",
    rmf2_scheduler::data::record_data_type::EVENT
  );
  py_utils::def_str_const(
    m_record_data_type,
    "TASK",
    rmf2_scheduler::data::record_data_type::TASK
  );
  py_utils::def_str_const(
    m_record_data_type,
    "PROCESS",
    rmf2_scheduler::data::record_data_type::PROCESS
  );
  py_utils::def_str_const(
    m_record_data_type,
    "SERIES",
    rmf2_scheduler::data::record_data_type::SERIES
  );

  // record_action_type
  py::module m_record_action_type = m_data.def_submodule("record_action_type");
  py_utils::def_str_const(
    m_record_action_type,
    "ADD",
    rmf2_scheduler::data::record_action_type::ADD
  );
  py_utils::def_str_const(
    m_record_action_type,
    "UPDATE",
    rmf2_scheduler::data::record_action_type::UPDATE
  );
  py_utils::def_str_const(
    m_record_action_type,
    "DELETE",
    rmf2_scheduler::data::record_action_type::DELETE
  );

  // ChangeAction
  py::class_<rmf2_scheduler::data::ChangeAction>(
    m_data,
    "ChangeAction",
    R"(
    ChangeActions for schedule
    )"
  )
  .def_readwrite(
    "id",
    &rmf2_scheduler::data::ChangeAction::id
  )
  .def_readwrite(
    "action",
    &rmf2_scheduler::data::ChangeAction::action
  )
  .def(py::self == py::self)
  .def(py::self != py::self)
  ;

  // ScheduleChangeRecord
  py::class_<rmf2_scheduler::data::ScheduleChangeRecord>(
    m_data,
    "ScheduleChangeRecord",
    R"(
    Records for the schedule changes
    )"
  )
  .def(
    "add",
    &rmf2_scheduler::data::ScheduleChangeRecord::add
  )
  .def(
    "get",
    &rmf2_scheduler::data::ScheduleChangeRecord::get
  )
  .def_static(
    "squash",
    &rmf2_scheduler::data::ScheduleChangeRecord::squash
  )
  ;
}

}  // namespace data

}  // namespace rmf2_scheduler_py
