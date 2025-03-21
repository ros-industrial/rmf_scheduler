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

#include <cstring>
#include <cstdlib>

#include "rmf2_scheduler/http/memory_stream.hpp"

namespace rmf2_scheduler
{

namespace http
{

MemoryStream::MemoryStream(size_t size)
{
  data_ = reinterpret_cast<char *>(malloc(size));
  capacity_ = size;
}

MemoryStream::~MemoryStream()
{
  free(data_);
}

Stream::UPtr MemoryStream::open_copy_of(
  const void * buffer,
  size_t size
)
{
  MemoryStream::UPtr stream = std::make_unique<MemoryStream>(size);
  // Copy the data over
  memcpy(stream->data_, buffer, size);
  stream->size_ = size;
  return stream;
}

Stream::UPtr MemoryStream::open_copy_of(
  const std::string & buffer
)
{
  return open_copy_of(buffer.c_str(), buffer.size());
}

Stream::UPtr MemoryStream::open_copy_of(
  const char * buffer
)
{
  return open_copy_of(buffer, std::strlen(buffer));
}

bool MemoryStream::set_position(size_t position) const
{
  return seek(position, Whence::FROM_BEGIN, nullptr);
}

bool MemoryStream::seek(
  size_t offset,
  Whence whence,
  size_t * new_position
) const
{
  size_t temp_position = 0;
  switch (whence) {
    case Whence::FROM_BEGIN:
      temp_position = offset;
      break;
    case Whence::FROM_CURRENT:
      temp_position = position_ + offset;
      break;
    case Whence::FROM_END:
      if (size_ < offset) {
        return false;
      }
      temp_position = size_ - offset;
      break;
  }

  if (temp_position > size_) {
    return false;
  }

  position_ = temp_position;

  if (new_position != nullptr) {
    *new_position = temp_position;
  }
  return true;
}

void MemoryStream::reserve(size_t size)
{
  if (size > capacity_) {
    data_ = realloc(data_, size);
    capacity_ = size;
  }
}

void MemoryStream::write(
  const void * buffer,
  size_t size_to_write,
  size_t * size_written
)
{
  if (position_ + size_to_write > capacity_) {
    reserve(position_ + size_to_write);
  }

  char * data_ptr = reinterpret_cast<char *>(data_);
  // TODO(anyone): handle out of memory

  // Copy the data over
  memcpy(&(data_ptr[position_]), buffer, size_to_write);
  if (size_written != nullptr) {
    *size_written = size_to_write;
  }

  // Update size and position
  position_ += size_to_write;
  if (position_ > size_) {
    size_ = position_;
  }
}

bool MemoryStream::read(
  void * buffer,
  size_t size_to_read,
  size_t * size_read
) const
{
  size_t actual_size_to_read = size_to_read;

  if (position_ + size_to_read > size_) {
    actual_size_to_read = size_ - position_;
  }

  if (actual_size_to_read > 0) {
    char * data_ptr = reinterpret_cast<char *>(data_);
    memcpy(buffer, data_ptr + position_, actual_size_to_read);

    // Update size and position
    position_ += actual_size_to_read;
  }

  if (size_read != nullptr) {
    *size_read = actual_size_to_read;
  }

  return true;
}

}  // namespace http
}  // namespace rmf2_scheduler
