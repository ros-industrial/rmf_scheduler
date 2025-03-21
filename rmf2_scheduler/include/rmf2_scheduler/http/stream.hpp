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

#ifndef RMF2_SCHEDULER__HTTP__STREAM_HPP_
#define RMF2_SCHEDULER__HTTP__STREAM_HPP_

#include <memory>

#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace http
{

class Stream
{
public:
  enum class Whence { FROM_BEGIN, FROM_CURRENT, FROM_END };

  RS_SMART_PTR_ALIASES_ONLY(Stream)

  Stream() = default;

  virtual ~Stream() = default;

  virtual size_t size() const = 0;

  virtual size_t capacity() const = 0;

  virtual size_t remaining_size() const = 0;

  virtual size_t position() const = 0;

  virtual bool set_position(size_t position) const = 0;

  virtual bool seek(
    size_t offset,
    Whence whence,
    size_t * new_position
  ) const = 0;

  virtual void reserve(size_t size) = 0;

  virtual void write(
    const void * buffer,
    size_t size_to_write,
    size_t * size_written
  ) = 0;

  virtual bool read(
    void * buffer,
    size_t size_to_read,
    size_t * size_read
  ) const = 0;

private:
  RS_DISABLE_COPY(Stream)
};

}  // namespace http

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__STREAM_HPP_
