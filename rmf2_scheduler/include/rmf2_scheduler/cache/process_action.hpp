// Copyright 2024 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__CACHE__PROCESS_ACTION_HPP_
#define RMF2_SCHEDULER__CACHE__PROCESS_ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rmf2_scheduler/cache/action.hpp"
#include "rmf2_scheduler/cache/task_handler.hpp"
#include "rmf2_scheduler/cache/process_handler.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace cache
{

class ProcessAction : public Action
{
public:
  RS_SMART_PTR_DEFINITIONS(ProcessAction)

  explicit ProcessAction(
    const std::string & type,
    const ActionPayload & payload
  );


  virtual ~ProcessAction();

  bool validate(
    const ScheduleCache::Ptr & cache,
    std::string & error
  ) override;

  void apply() override;

private:
  RS_DISABLE_COPY(ProcessAction)

  bool _validate_process_add(
    const ProcessHandler::ConstPtr & process_handler,
    const TaskHandler::ConstPtr & task_handler,
    std::string & error
  );

  void _apply_process_add();

  bool _validate_process_attach_node(
    const ProcessHandler::ConstPtr & process_handler,
    const TaskHandler::ConstPtr & task_handler,
    std::string & error
  );

  void _apply_process_attach_node();

  bool _validate_process_add_dependency(
    const ProcessHandler::ConstPtr & process_handler,
    const TaskHandler::ConstPtr & task_handler,
    std::string & error
  );

  void _apply_process_add_dependency();

  bool _validate_process_update(
    const ProcessHandler::ConstPtr & process_handler,
    const TaskHandler::ConstPtr & task_handler,
    std::string & error
  );

  void _apply_process_update();

  bool _validate_process_update_start_time(
    const ProcessHandler::ConstPtr & process_handler,
    std::string & error
  );

  void _apply_process_update_start_time();

  bool _validate_process_detach_node(
    const ProcessHandler::ConstPtr & process_handler,
    const TaskHandler::ConstPtr & task_handler,
    std::string & error
  );

  void _apply_process_detach_node();

  bool _validate_process_delete_dependency(
    const ProcessHandler::ConstPtr & process_handler,
    const TaskHandler::ConstPtr & task_handler,
    std::string & error
  );

  void _apply_process_delete_dependency();

  bool _validate_process_delete(
    const ProcessHandler::ConstPtr & process_handler,
    const TaskHandler::ConstPtr & task_handler,
    std::string & error
  );

  void _apply_process_delete();

  void _apply_process_delete_all();

  ProcessHandler::Ptr process_handler_;
  TaskHandler::Ptr task_handler_;

  ProcessHandler::ProcessConstIterator process_itr_;
  std::vector<TaskHandler::TaskConstIterator> task_itrs_;
  std::vector<TaskHandler::TaskConstIterator> extra_task_itrs_;
};

}  // namespace cache
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__CACHE__PROCESS_ACTION_HPP_
