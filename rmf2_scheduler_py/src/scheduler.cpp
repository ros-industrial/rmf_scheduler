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

#include "rmf2_scheduler_py/scheduler.hpp"
#include "rmf2_scheduler/scheduler.hpp"

namespace rmf2_scheduler_py
{

void init_scheduler_py(py::module & m)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  py::class_<Optimizer, Optimizer::Ptr>(
    m,
    "Optimizer"
  );

  py::class_<Estimator, Estimator::Ptr>(
    m,
    "Estimator"
  );

  py::class_<Scheduler, Scheduler::Ptr>(
    m,
    "Scheduler",
    R"(
    Scheduler Class
    )"
  )
  .def(
    py::init<
      SchedulerOptions::Ptr,
      storage::ScheduleStream::Ptr,
      SystemTimeExecutor::Ptr,
      Optimizer::Ptr,
      Estimator::Ptr,
      ProcessExecutor::Ptr,
      TaskExecutorManager::Ptr
    >(),
    py::arg("options"),
    py::arg("stream"),
    py::arg("system_time_executor"),
    py::arg("optimizer") = Optimizer::Ptr(),
    py::arg("estimator") = Estimator::Ptr(),
    py::arg("process_executor") = ProcessExecutor::Ptr(),
    py::arg("task_executor_manager") = TaskExecutorManager::Ptr()
  )
  .def(
    "get_schedule",
    [](
      Scheduler & self,
      const data::Time & start_time,
      const data::Time & end_time,
      int offset,
      int limit
    ) {
      std::string error;
      std::vector<data::Task::ConstPtr> tasks;
      std::vector<data::Process::ConstPtr> processes;
      std::vector<data::Series::ConstPtr> serieses;

      bool result = self.get_schedule(
        start_time,
        end_time,
        offset,
        limit,
        tasks,
        processes,
        serieses,
        error
      );

      return py::make_tuple(result, error, tasks, processes, serieses);
    }
  )
  .def(
    "validate",
    [](
      Scheduler & self,
      cache::Action::Ptr action
    ) {
      std::string error;
      bool result = self.validate(action, error);

      return py::make_tuple(result, error);
    }
  )
  .def(
    "validate",
    [](
      Scheduler & self,
      const data::ScheduleAction & action
    ) {
      std::string error;
      bool result = self.validate(action, error);

      return py::make_tuple(result, error);
    }
  )
  .def(
    "perform",
    [](
      Scheduler & self,
      cache::Action::Ptr action
    ) {
      std::string error;
      bool result = self.perform(action, error);

      return py::make_tuple(result, error);
    }
  )
  .def(
    "perform",
    [](
      Scheduler & self,
      const data::ScheduleAction & action
    ) {
      std::string error;
      bool result = self.perform(action, error);

      return py::make_tuple(result, error);
    }
  )
  .def(
    "validate_batch",
    [](
      Scheduler & self,
      const std::vector<cache::Action::Ptr> & actions
    ) {
      std::string error;
      bool result = self.validate_batch(actions, error);
      return py::make_tuple(result, error);
    }
  )
  .def(
    "validate_batch",
    [](
      Scheduler & self,
      const std::vector<data::ScheduleAction> & actions
    ) {
      std::string error;
      bool result = self.validate_batch(actions, error);
      return py::make_tuple(result, error);
    }
  )
  .def(
    "perform_batch",
    [](
      Scheduler & self,
      const std::vector<cache::Action::Ptr> & actions
    ) {
      std::string error;
      bool result = self.perform_batch(actions, error);
      return py::make_tuple(result, error);
    }
  )
  .def(
    "perform_batch",
    [](
      Scheduler & self,
      const std::vector<data::ScheduleAction> & actions
    ) {
      std::string error;
      bool result = self.perform_batch(actions, error);
      return py::make_tuple(result, error);
    }
  )
  ;


  py::class_<LockedScheduleRO>(
    m,
    "LockedScheduleRO",
    R"(
    Acquire read-only schedule cache from the scheduler
    )"
  )
  .def(
    py::init<const Scheduler::ConstPtr &>(),
    py::arg("scheduler")
  )
  .def(
    "cache",
    &LockedScheduleRO::cache
  )
  ;

  py::class_<LockedScheduleRW>(
    m,
    "LockedScheduleRW",
    R"(
    Acquire read-write schedule from the scheduler
    )"
  )
  .def(
    py::init<const Scheduler::Ptr &>(),
    py::arg("scheduler")
  )
  .def(
    "cache",
    &LockedScheduleRW::cache
  )
  ;
}

}  // namespace rmf2_scheduler_py
