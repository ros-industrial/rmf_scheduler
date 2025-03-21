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

#ifndef RMF2_SCHEDULER__HTTP__MOCK_TRANSPORT_HPP_
#define RMF2_SCHEDULER__HTTP__MOCK_TRANSPORT_HPP_

#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "rmf2_scheduler/http/transport.hpp"

namespace rmf2_scheduler
{

namespace http
{

class MockTransport : public Transport
{
public:
  MockTransport() = default;
  virtual ~MockTransport() = default;
  MOCK_METHOD(
    std::shared_ptr<Connection>,
    create_connection,
    (
      const std::string &,
      const std::string &,
      const HeaderList &,
      const std::string &,
      const std::string &,
      std::string &
    ),
    (override))
  ;

private:
  RS_DISABLE_COPY(MockTransport)
};

}  // namespace http

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__MOCK_TRANSPORT_HPP_
