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

#ifndef RMF2_SCHEDULER__HTTP__MOCK_STREAM_HPP_
#define RMF2_SCHEDULER__HTTP__MOCK_STREAM_HPP_

#include <memory>

#include "gmock/gmock.h"
#include "rmf2_scheduler/http/stream.hpp"

namespace rmf2_scheduler
{

namespace http
{

class MockStream : public Stream
{
public:
  MockStream() = default;
  virtual ~MockStream() = default;
  MOCK_METHOD(size_t, size, (), (const, override));
  MOCK_METHOD(size_t, capacity, (), (const, override));
  MOCK_METHOD(size_t, remaining_size, (), (const, override));
  MOCK_METHOD(size_t, position, (), (const, override));
  MOCK_METHOD(bool, set_position, (size_t), (const, override));
  MOCK_METHOD(bool, seek, (size_t, Whence, size_t *), (const, override));
  MOCK_METHOD(void, reserve, (size_t), (override));
  MOCK_METHOD(void, write, (const void *, size_t, size_t *), (override));
  MOCK_METHOD(bool, read, (void *, size_t, size_t *), (const, override));

private:
  RS_DISABLE_COPY(MockStream)
};

}  // namespace http

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__MOCK_STREAM_HPP_
