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

#ifndef RMF2_SCHEDULER__HTTP__CONNECTION_FAKE_HPP_
#define RMF2_SCHEDULER__HTTP__CONNECTION_FAKE_HPP_

#include <cstring>
#include <map>
#include <memory>
#include <string>

#include "rmf2_scheduler/http/connection.hpp"
#include "rmf2_scheduler/http/transport_fake.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace http
{

namespace fake
{

class Connection : public http::Connection
{
public:
  explicit Connection(
    const std::string & url,
    const std::string & method,
    const HandlerCallback & handler
  );

  ~Connection() override;

  bool send_headers(
    const HeaderList & headers,
    std::string & error
  ) override;

  bool set_request_data(
    Stream::UPtr data,
    std::string & error
  ) override;

  bool set_response_data(Stream::UPtr /* data */) override {return true;}

  bool perform_request(std::string & error_text) override;

  int get_response_status_code() const override;

  std::string get_response_status_text() const override;

  std::string get_protocol_version() const override;

  std::string get_response_header(
    const std::string & header_name
  ) const override;

  Stream::UPtr extract_data_stream() override;

private:
  RS_DISABLE_COPY(Connection)

  HandlerCallback handler_;

  // Request and response objects passed to the user-provided request handler
  // callback. The request object contains all the request information.
  // The response object is the server response that is created by
  // the handler in response to the request.
  ServerRequest request_;
  ServerResponse response_;
};

}  // namespace fake

}  // namespace http

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__CONNECTION_FAKE_HPP_
