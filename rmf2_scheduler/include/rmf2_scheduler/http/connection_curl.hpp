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

#ifndef RMF2_SCHEDULER__HTTP__CONNECTION_CURL_HPP_
#define RMF2_SCHEDULER__HTTP__CONNECTION_CURL_HPP_

#include <cstring>
#include <map>
#include <memory>
#include <string>

#include "curl/curl.h"
#include "rmf2_scheduler/http/connection.hpp"
#include "rmf2_scheduler/http/curl_api.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace http
{

namespace curl
{

class Connection : public http::Connection
{
public:
  Connection(
    CURL * curl_handle,
    const std::string & method,
    const std::shared_ptr<CURLInterface> & curl_interface
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

  bool set_response_data(Stream::UPtr data) override;

  bool perform_request(std::string & error_text) override;

  int get_response_status_code() const override;

  std::string get_response_status_text() const override;

  std::string get_protocol_version() const override;

  std::string get_response_header(
    const std::string & header_name
  ) const override;

  Stream::UPtr extract_data_stream() override;

protected:
  // Write data callback. Used by CURL when receiving response data.
  static size_t write_callback(
    char * ptr,
    size_t size,
    size_t num,
    void * data
  );

  // Read data callback. Used by CURL when sending request body data.
  static size_t read_callback(
    char * ptr,
    size_t size,
    size_t num,
    void * data
  );

  // Write header data callback. Used by CURL when receiving response headers.
  static size_t header_callback(
    char * ptr,
    size_t size,
    size_t num,
    void * data
  );

  void prepare_request();

  CURL * curl_handle_;
  curl_slist * header_list_ = nullptr;

  std::string method_;
  std::shared_ptr<CURLInterface> curl_interface_;

  Stream::UPtr request_buffer_;
  Stream::UPtr response_buffer_;

  // List of optional request headers provided by the caller.
  std::multimap<std::string, std::string> request_headers_;

  // List of headers received in response. Key saved as lower casing for later
  // processing as headers should be case-insensitive.
  std::multimap<std::string, std::string> response_headers_;

  // HTTP protocol version, such as HTTP/1.1
  std::string protocol_version_;

  // Response status text, such as "OK" for 200, or "Forbidden" for 403
  std::string status_text_;
  // Flag used when parsing response headers to separate the response status
  // from the rest of response headers.
  bool status_text_set_ = false;

private:
  RS_DISABLE_COPY(Connection)
};

}  // namespace curl

}  // namespace http

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__CONNECTION_CURL_HPP_
