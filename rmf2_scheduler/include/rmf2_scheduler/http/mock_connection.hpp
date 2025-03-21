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

#ifndef RMF2_SCHEDULER__HTTP__MOCK_CONNECTION_HPP_
#define RMF2_SCHEDULER__HTTP__MOCK_CONNECTION_HPP_

#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "rmf2_scheduler/http/connection.hpp"

namespace rmf2_scheduler
{

namespace http
{

class MockConnection : public Connection
{
public:
  MockConnection() = default;
  virtual ~MockConnection() = default;

  MOCK_METHOD(bool, send_headers, (const HeaderList &, std::string &), (override));
  MOCK_METHOD(bool, set_request_data, (Stream::UPtr, std::string &), (override));
  MOCK_METHOD(bool, set_response_data, (Stream::UPtr), (override));
  MOCK_METHOD(bool, perform_request, (std::string &), (override));
  MOCK_METHOD(int, get_response_status_code, (), (const, override));
  MOCK_METHOD(std::string, get_response_status_text, (), (const, override));
  MOCK_METHOD(std::string, get_protocol_version, (), (const, override));
  MOCK_METHOD(std::string, get_response_header, (const std::string &), (const, override));
  MOCK_METHOD(Stream::UPtr, extract_data_stream, (), (override));

private:
  RS_DISABLE_COPY(MockConnection)
};

}  // namespace http

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__MOCK_CONNECTION_HPP_
