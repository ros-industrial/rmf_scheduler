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

#ifndef RMF2_SCHEDULER__STORAGE__SCHEDULE_STREAM_LD_BROKER_HPP_
#define RMF2_SCHEDULER__STORAGE__SCHEDULE_STREAM_LD_BROKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rmf2_scheduler/storage/schedule_stream.hpp"
#include "rmf2_scheduler/http/transport.hpp"

namespace rmf2_scheduler
{

namespace storage
{

namespace ld_broker
{

class ScheduleStream : public storage::ScheduleStream
{
public:
  RS_SMART_PTR_ALIASES_ONLY(ScheduleStream)

  ScheduleStream(
    const std::string & ld_broker_url,
    std::shared_ptr<http::Transport> transport
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

private:
  bool _read_response_to_tasks(
    const std::string & response_stream,
    std::vector<data::Task::Ptr> & tasks,
    std::string & error
  ) const;

  bool _read_response_to_processes(
    const std::string & response_stream,
    std::vector<data::Process::Ptr> & processes,
    std::string & error
  ) const;

  std::string create_url_;
  std::string read_url_;
  std::string upsert_url_;
  std::string update_url_;
  std::string delete_url_;

  std::shared_ptr<http::Transport> transport_;
};

}  // namespace ld_broker
}  // namespace storage
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__STORAGE__SCHEDULE_STREAM_LD_BROKER_HPP_
