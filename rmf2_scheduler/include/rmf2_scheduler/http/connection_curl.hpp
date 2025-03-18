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

class StreamBuffer
{
public:
  explicit StreamBuffer(size_t size)
  {
    buffer_ = reinterpret_cast<char *>(malloc(size + 1));
    buffer_length_ = size + 1;
    buffer_[0] = '\0';
  }

  std::string str()
  {
    return std::string(buffer_);
  }

  void write(const char * data_ptr, size_t data_len)
  {
    char * buffer_ptr;
    size_t old_size = str_size();
    size_t new_size = buffer_length_;
    if (old_size + data_len + 1 > buffer_length_) {
      buffer_ptr = reinterpret_cast<char *>(
        realloc(buffer_, old_size + data_len + 1)
      );
      new_size = old_size + data_len + 1;
    } else {
      buffer_ptr = buffer_;
    }

    // TODO(anyone): handle out of memory

    // Copy the string and null terminate
    buffer_ = buffer_ptr;
    memcpy(&(buffer_[old_size]), data_ptr, data_len);
    buffer_[old_size + data_len] = '\0';
    buffer_length_ = new_size;
  }

  bool read(
    char * data_ptr,
    size_t data_len,
    size_t & read_size
  )
  {
    if (seek + data_len >= str_size()) {
      read_size = str_size() - seek;
    } else {
      read_size = data_len;
    }
    memcpy(data_ptr, buffer_ + seek, read_size);
    seek += read_size;
    return read_size <= data_len;
  }

  size_t str_size()
  {
    return strlen(buffer_);
  }

  ~StreamBuffer()
  {
    free(buffer_);
  }

private:
  size_t seek = 0;
  char * buffer_ = nullptr;
  size_t buffer_length_ = 0;
};

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
    const std::string & data,
    std::string & error
  ) override;

  bool perform_request(std::string & error_text) override;

  int get_response_status_code() const override;

  std::string get_response_status_text() const override;

  std::string get_protocol_version() const override;

  std::string get_response_header(
    const std::string & header_name
  ) const override;

  std::string extract_data_stream() override;

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

  StreamBuffer request_buffer_;
  StreamBuffer response_buffer_;

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
