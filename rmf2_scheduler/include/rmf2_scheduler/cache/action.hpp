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

#ifndef RMF2_SCHEDULER__CACHE__ACTION_HPP_
#define RMF2_SCHEDULER__CACHE__ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rmf2_scheduler/data/schedule_action.hpp"
#include "rmf2_scheduler/data/schedule_change_record.hpp"
#include "rmf2_scheduler/cache/schedule_cache.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace cache
{

class ActionPayload
{
public:
  ActionPayload() = default;
  virtual ~ActionPayload() = default;

  ActionPayload & id(const std::string & id)
  {
    data_.id = id;
    return *this;
  }

  ActionPayload & event(const data::Event::Ptr & event)
  {
    data_.event = data::Event::make_shared(*event);
    return *this;
  }

  ActionPayload & task(const data::Task::Ptr & task)
  {
    data_.task = data::Task::make_shared(*task);
    return *this;
  }

  ActionPayload & process(const data::Process::Ptr & process)
  {
    data_.process = data::Process::make_shared(*process);
    return *this;
  }

  ActionPayload & node_id(const std::string & node_id)
  {
    data_.node_id = node_id;
    return *this;
  }

  ActionPayload & source_id(const std::string & source_id)
  {
    data_.source_id = source_id;
    return *this;
  }

  ActionPayload & destination_id(const std::string & destination_id)
  {
    data_.destination_id = destination_id;
    return *this;
  }

  ActionPayload & edge_type(const std::string & edge_type)
  {
    data_.edge_type = edge_type;
    return *this;
  }

  data::ScheduleAction data(
    const std::string & type
  ) const
  {
    auto result = data_;
    result.type = type;
    return result;
  }

private:
  data::ScheduleAction data_;

  friend class Action;
};

class Action
{
public:
  RS_SMART_PTR_DEFINITIONS(Action)

  explicit Action(
    const std::string & type,
    const ActionPayload & payload
  )
  : data_(payload.data(type))
  {
  }

  virtual ~Action()
  {
  }

  const data::ScheduleAction & data() const
  {
    return data_;
  }

  const data::ScheduleChangeRecord & record() const
  {
    return record_;
  }

  virtual bool validate(
    const ScheduleCache::Ptr & scheduler_cache,
    std::string & error
  ) = 0;

  virtual void apply() = 0;

  static std::shared_ptr<Action> create(
    const std::string & type,
    const ActionPayload & payload
  );

  static std::shared_ptr<Action> create(
    const data::ScheduleAction & data
  );

  bool is_valid() const
  {
    return valid_;
  }

  bool is_applied() const
  {
    return applied_;
  }

protected:
  bool on_validate_prepare(std::string & error)
  {
    if (is_applied()) {
      error = "Already applied!";
      return false;
    }

    if (is_valid()) {
      error = "Already valid!";
      return false;
    }

    return true;
  }

  void on_apply_prepare()
  {
    if (is_applied()) {
      throw std::runtime_error("Already applied!");
    }

    if (!is_valid()) {
      throw std::runtime_error("Action invalid!");
    }
  }

  void on_validate_complete()
  {
    valid_ = true;
  }

  void on_apply_complete()
  {
    applied_ = true;
  }

  data::ScheduleChangeRecord record_;
  data::ScheduleAction data_;

private:
  RS_DISABLE_COPY(Action)

  bool valid_ = false;
  bool applied_ = false;
};

}  // namespace cache
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__CACHE__ACTION_HPP_
