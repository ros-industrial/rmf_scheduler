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

#ifndef RMF2_SCHEDULER__CACHE__SERIES_ACTION_HPP_
#define RMF2_SCHEDULER__CACHE__SERIES_ACTION_HPP_

#include <memory>
#include <string>

#include "rmf2_scheduler/cache/action.hpp"
#include "rmf2_scheduler/cache/schedule_cache.hpp"
#include "rmf2_scheduler/cache/series_handler.hpp"
#include "rmf2_scheduler/cache/task_handler.hpp"
#include "rmf2_scheduler/cache/process_handler.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace cache
{

class SeriesAction : public Action
{
public:
  RS_SMART_PTR_DEFINITIONS(SeriesAction)

  explicit SeriesAction(
    const std::string & type,
    const ActionPayload & payload
  );

  virtual ~SeriesAction();

  bool validate(
    const ScheduleCache::Ptr & cache,
    std::string & error
  ) override;

  void apply() override;

private:
  RS_DISABLE_COPY(SeriesAction)

  bool _validate_series_add(
    const SeriesHandler::ConstPtr & series_handler,
    const TaskHandler::ConstPtr & task_handler,
    const ProcessHandler::ConstPtr & process_handler,
    std::string & error
  );

  // CURRENTLY NOT SUPPORTED
  bool _validate_series_update(
    const SeriesHandler::ConstPtr & series_handler,
    std::string & error
  );

  bool _validate_series_expand_until(
    const SeriesHandler::ConstPtr & series_handler,
    std::string & error
  );

  bool _validate_series_update_cron(
    const SeriesHandler::ConstPtr & series_handler,
    std::string & error
  );

  // CURRENTLY NOT SUPPORTED
  bool _validate_series_update_until(
    const SeriesHandler::ConstPtr & series_handler,
    std::string & error
  );

  // CURRENTLY NOT SUPPORTED
  bool _validate_series_update_occurrence(
    const SeriesHandler::ConstPtr & series_handler,
    std::string & error
  );

  bool _validate_series_update_occurrence_time(
    const SeriesHandler::ConstPtr & series_handler,
    const TaskHandler::ConstPtr & task_handler,
    const ProcessHandler::ConstPtr & process_handler,
    std::string & error
  );

  bool _validate_series_delete_occurrence(
    const SeriesHandler::ConstPtr & series_handler,
    const TaskHandler::ConstPtr & task_handler,
    const ProcessHandler::ConstPtr & process_handler,
    std::string & error
  );

  bool _validate_series_delete(
    const SeriesHandler::ConstPtr & task_handler,
    std::string & error
  );

  bool _validate_series_delete_all(
    const SeriesHandler::ConstPtr & task_handler,
    std::string & error
  );

  void _apply_series_add();

  void _apply_series_update();

  void _apply_series_expand_until();

  void _apply_series_update_cron();

  void _apply_series_update_until();

  void _apply_series_update_occurrence();

  void _apply_series_update_occurrence_time();

  void _apply_series_delete_occurrence();

  void _apply_series_delete_series();

  void _apply_series_delete_all();


  SeriesHandler::Ptr series_handler_;
  TaskHandler::Ptr task_handler_;
  ProcessHandler::Ptr process_handler_;
};

}  // namespace cache
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__CACHE__SERIES_ACTION_HPP_
