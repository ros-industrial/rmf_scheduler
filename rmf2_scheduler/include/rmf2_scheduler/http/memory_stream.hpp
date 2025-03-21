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

#ifndef RMF2_SCHEDULER__HTTP__MEMORY_STREAM_HPP_
#define RMF2_SCHEDULER__HTTP__MEMORY_STREAM_HPP_

#include <memory>
#include <string>

#include "rmf2_scheduler/http/stream.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace http
{

class MemoryStream : public Stream
{
public:
  RS_SMART_PTR_ALIASES_ONLY(MemoryStream)

  explicit MemoryStream(size_t size);

  static Stream::UPtr open_copy_of(const void * buffer, size_t size);
  static Stream::UPtr open_copy_of(const std::string & buffer);
  static Stream::UPtr open_copy_of(const char * buffer);

  size_t size() const override {return size_;}

  size_t capacity() const override {return capacity_;}

  size_t remaining_size() const override {return size_ - position_;}

  size_t position() const override {return position_;}

  bool set_position(size_t position) const override;

  bool seek(
    size_t offset,
    Whence whence,
    size_t * new_position
  ) const override;

  void reserve(size_t size) override;
  void write(
    const void * buffer,
    size_t size_to_write,
    size_t * size_written
  ) override;

  bool read(
    void * buffer,
    size_t size_to_read,
    size_t * size_read
  ) const override;

  virtual ~MemoryStream();

private:
  mutable size_t position_ = 0;
  void * data_ = nullptr;
  size_t size_ = 0;
  size_t capacity_ = 0;
};

}  // namespace http

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__MEMORY_STREAM_HPP_
