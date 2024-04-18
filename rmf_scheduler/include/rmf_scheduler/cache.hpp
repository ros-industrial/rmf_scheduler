// Copyright 2023 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF_SCHEDULER__CACHE_HPP_
#define RMF_SCHEDULER__CACHE_HPP_

#include <deque>
#include <string>
#include <utility>

#include "rmf_scheduler/data/schedule.hpp"

namespace rmf_scheduler
{

class Cache
{
public:
  explicit Cache(
    size_t keep_last,
    std::string cache_dir);

  void from_last_cache(data::Schedule & schedule);

  void write_local_cache(const data::Schedule & schedule);

private:
  void load_cache(
    const std::string & uuid,
    data::Schedule & schedule);
  void remove_cache(const std::string & uuid);

  std::string cache_dir_;
  std::deque<std::string> cache_ids_;
};

namespace exception
{

class CacheInvalidException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  CacheInvalidException(const char * msg, Args && ... args)
  : ExceptionTemplate(
      ErrorCode::FAILURE,
      msg, std::forward<Args>(args) ...)
  {
    add_prefix("CacheInvalidException:\n  ");
  }
};
}  // namespace exception

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__CACHE_HPP_
