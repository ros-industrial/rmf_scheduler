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

#ifndef RMF2_SCHEDULER__STORAGE__SCHEDULE_STREAM_SIMPLE_HPP_
#define RMF2_SCHEDULER__STORAGE__SCHEDULE_STREAM_SIMPLE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <deque>

#include "rmf2_scheduler/storage/schedule_stream.hpp"
#include "rmf2_scheduler/http/transport.hpp"

namespace rmf2_scheduler
{

namespace storage
{

namespace simple
{

class ScheduleStream : public storage::ScheduleStream
{
public:
  RS_SMART_PTR_ALIASES_ONLY(ScheduleStream)

  ScheduleStream(
    size_t keep_last,
    const std::string & backup_folder_path
  );

  virtual ~ScheduleStream();

  bool read_schedule(
    cache::ScheduleCache::Ptr cache,
    const data::TimeWindow & time_window,
    std::string & error
  ) override;

  bool write_schedule(
    cache::ScheduleCache::ConstPtr cache,
    const data::TimeWindow & time_window,
    std::string & error
  ) override;

  bool write_schedule(
    cache::ScheduleCache::ConstPtr cache,
    const std::vector<data::ScheduleChangeRecord> & change_actions,
    std::string & error
  ) override;

  bool refresh_tasks(
    cache::ScheduleCache::Ptr cache,
    const std::vector<std::string> & ids,
    std::string & error
  ) override;

private:
  void _remove_backup(const std::string & uuid);
  bool _load_backup(
    const std::string & uuid,
    cache::ScheduleCache::Ptr schedule,
    std::string & error
  );

  void _write_schedule_to_backup(cache::ScheduleCache::ConstPtr schedule);

  size_t keep_last_;
  std::string backup_folder_path_;

  std::deque<std::string> backup_ids_;
};

}  // namespace simple
}  // namespace storage
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__STORAGE__SCHEDULE_STREAM_SIMPLE_HPP_
