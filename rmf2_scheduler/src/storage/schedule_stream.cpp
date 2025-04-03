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

#include "rmf2_scheduler/storage/schedule_stream.hpp"
#include "rmf2_scheduler/storage/schedule_stream_ld_broker.hpp"
#include "rmf2_scheduler/storage/schedule_stream_simple.hpp"

namespace rmf2_scheduler
{

namespace storage
{

ScheduleStream::Ptr ScheduleStream::create_default(
  const std::string & url
)
{
  return std::make_shared<ld_broker::ScheduleStream>(
    url,
    http::Transport::create_default()
  );
}

ScheduleStream::Ptr ScheduleStream::create_simple(
  size_t keep_last,
  const std::string & backup_folder_path
)
{
  return std::make_shared<storage::simple::ScheduleStream>(
    keep_last,
    backup_folder_path
  );
}

}  // namespace storage
}  // namespace rmf2_scheduler
