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

#ifndef RMF2_SCHEDULER__HTTP__REQUEST_HPP_
#define RMF2_SCHEDULER__HTTP__REQUEST_HPP_

#include <map>
#include <memory>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "rmf2_scheduler/http/common.hpp"
#include "rmf2_scheduler/http/transport.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace http
{

class Response;

class Request final
{
public:
  // The main constructor. |url| specifies the remote host address/path
  // to send the request to. |method| is the HTTP request verb and
  // |transport| is the HTTP transport implementation for server communications.
  Request(
    const std::string & url,
    const std::string & method,
    std::shared_ptr<Transport> transport
  );

  ~Request();

  // Gets/Sets "Accept:" header value. The default value is "*/*" if not set.
  void set_accept(const std::string & accept_mime_types);
  const std::string & get_accept() const;

  // Gets/Sets "Content-Type:" header value
  void set_content_type(const std::string & content_type);
  const std::string & get_content_type() const;

  // Adds additional HTTP request header
  void add_header(const std::string & header, const std::string & value);
  void add_headers(const HeaderList & headers);

  // Removes HTTP request header
  void remove_header(const std::string & header);

  // Adds a request body. This is not to be used with GET method
  bool add_request_body(const void * data, size_t size, std::string & error);
  bool add_request_body(Stream::UPtr stream, std::string & error);
  bool add_request_body(const nlohmann::json & json, std::string & error);

  // // Adds a request body. This is not to be used with GET method.
  // // This method also sets the correct content-type of the request, including
  // // the multipart data boundary.
  // bool AddRequestBodyAsFormData(std::unique_ptr<FormData> form_data,
  //                               brillo::ErrorPtr* error);

  // Adds a stream for the response. Otherwise a MemoryStream will be used.
  bool add_response_stream(Stream::UPtr stream, std::string & error);

  // Makes a request for a subrange of data. Specifies a partial range with
  // either from beginning of the data to the specified offset (if |bytes| is
  // negative) or from the specified offset to the end of data (if |bytes| is
  // positive).
  // All individual ranges will be sent as part of "Range:" HTTP request header.
  void add_range(int64_t bytes);
  // Makes a request for a subrange of data. Specifies a full range with
  // start and end bytes from the beginning of the requested data.
  // All individual ranges will be sent as part of "Range:" HTTP request header.
  void add_range(uint64_t from_byte, uint64_t to_byte);

  // Returns the request URL
  const std::string & get_request_url() const;

  // Returns the request verb.
  const std::string & get_request_method() const;

  // Gets/Sets a request referer URL (sent as "Referer:" request header).
  void set_referer(const std::string & referer);
  const std::string & get_referer() const;

  // Gets/Sets a user agent string (sent as "User-Agent:" request header).
  void set_user_agent(const std::string & user_agent);
  const std::string & get_user_agent() const;

  // Sends the request to the server and blocks until the response is received,
  // which is returned as the response object.
  // In case the server couldn't be reached for whatever reason, returns
  // empty unique_ptr (null). In such a case, the additional error information
  // can be returned through the optional supplied |error| parameter.
  std::unique_ptr<Response> get_response_and_block(std::string & error);

private:
  RS_DISABLE_COPY(Request)

  // Helper function to create an http::Connection and send off request headers.
  bool _send_request_if_needed(std::string & error);

  // Implementation that provides particular HTTP transport.
  std::shared_ptr<Transport> transport_;
  // An established connection for adding request body. This connection
  // is maintained by the request object after the headers have been
  // sent and before the response is requested.
  std::shared_ptr<Connection> connection_;
  // Full request URL, such as "http://www.host.com/path/to/object"
  const std::string request_url_;
  // HTTP request verb, such as "GET", "POST", "PUT", ...
  const std::string method_;
  // Referrer URL, if any. Sent to the server via "Referer: " header.
  std::string referer_;
  // User agent string, if any. Sent to the server via "User-Agent: " header.
  std::string user_agent_;
  // Content type of the request body data.
  // Sent to the server via "Content-Type: " header.
  std::string content_type_;
  // List of acceptable response data types.
  // Sent to the server via "Accept: " header.
  std::string accept_ = "*/*";
  // List of optional request headers provided by the caller.
  std::multimap<std::string, std::string> headers_;

  // List of optional data ranges to request partial content from the server.
  // Sent to the server as "Range: " header.
  std::vector<std::pair<uint64_t, uint64_t>> ranges_;
  // range_value_omitted is used in |ranges_| list to indicate omitted value.
  // E.g. range (10,range_value_omitted) represents bytes from 10 to the end
  // of the data stream.
  const uint64_t range_value_omitted = std::numeric_limits<uint64_t>::max();
};

class Response final
{
public:
  explicit Response(const std::shared_ptr<Connection> & connection);
  ~Response();

  // Returns true if server returned a success code (status code below 400).
  bool is_successful() const;

  // Returns the HTTP status code (e.g. 200 for success)
  int get_status_code() const;

  // Returns the status text (e.g. for error 403 it could be "NOT AUTHORIZED").
  std::string get_status_text() const;

  // Returns the content type of the response data.
  std::string get_content_type() const;

  // Returns response data stream by transferring ownership of the data stream
  // from Response class to the caller.
  Stream::UPtr extract_data_stream();

  // Extracts the data from the underlying response data stream as a byte array.
  std::vector<uint8_t> extract_data();

  // Extracts the data from the underlying response data stream as a string.
  std::string extract_data_as_string();

  // Returns a value of a given response HTTP header.
  std::string get_header(const std::string & header_name) const;

private:
  RS_DISABLE_COPY(Response)

  std::shared_ptr<Connection> connection_;
};

}  // namespace http

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__REQUEST_HPP_
