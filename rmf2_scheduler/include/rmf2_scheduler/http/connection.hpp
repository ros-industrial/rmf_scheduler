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

#ifndef RMF2_SCHEDULER__HTTP__CONNECTION_HPP_
#define RMF2_SCHEDULER__HTTP__CONNECTION_HPP_

#include <memory>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace http
{

class Connection
{
public:
  using HeaderList = std::vector<std::pair<std::string, std::string>>;

  Connection() = default;
  virtual ~Connection() = default;

  // The following methods are used by http::Request object to initiate the
  // communication with the server, send the request data and receive the
  // response.
  // Called by http::Request to initiate the connection with the server.
  // This normally opens the socket and sends the request headers.
  virtual bool send_headers(
    const HeaderList & headers,
    std::string & error
  ) = 0;

  virtual bool set_request_data(
    const std::string & data,
    std::string & error
  ) = 0;

  virtual bool perform_request(std::string & error_text) = 0;

  /// Returns the HTTP status code (e.g. 200 for success).
  virtual int get_response_status_code() const = 0;

  /// Returns the status text (e.g. for error 403 it could be "NOT AUTHORIZED").
  virtual std::string get_response_status_text() const = 0;

  /// Returns the HTTP protocol version (e.g. "HTTP/1.1").
  virtual std::string get_protocol_version() const = 0;

  /// Returns the value of particular response header, or empty string if the
  /// headers wasn't received.
  virtual std::string get_response_header(
    const std::string & header_name
  ) const = 0;

  // Returns the response data stream. This function can be called only once
  // as it transfers ownership of the data stream to the caller. Subsequent
  // calls will fail with "Stream closed" error.
  // Returns empty stream on failure and fills in the error information in
  // |error| object.
  virtual std::string extract_data_stream() = 0;

private:
  RS_DISABLE_COPY(Connection)
};

}  // namespace http
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__CONNECTION_HPP_
