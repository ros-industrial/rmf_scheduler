// Copyright 2025 ROS Industrial Consortium Asia Pacific
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

#include "rmf2_scheduler/http/connection_fake.hpp"
#include "rmf2_scheduler/http/common.hpp"
#include "rmf2_scheduler/http/memory_stream.hpp"
#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler
{

namespace http
{

namespace fake
{

Connection::Connection(
  const std::string & url,
  const std::string & method,
  const HandlerCallback & handler
)
: handler_(handler), request_(url, method)
{
  LOG_DEBUG("fake::Connection created: %s", method.c_str());
}

Connection::~Connection()
{
  LOG_DEBUG("fake::Connection destroyed.");
}

bool Connection::send_headers(
  const HeaderList & headers,
  std::string & /*error*/
)
{
  request_.add_headers(headers);
  return true;
}

bool Connection::set_request_data(
  Stream::UPtr stream,
  std::string & /*error*/
)
{
  request_.set_data(std::move(stream));
  return true;
}

bool Connection::perform_request(std::string & /*error*/)
{
  request_.add_headers(
        {
          {request_header::kContentLength, std::to_string(request_.get_data().size())}
        });

  if (!handler_) {
    LOG_ERROR(
      "Received unexpected %s request at %s",
      request_.get_method().c_str(),
      request_.get_url().c_str()
    );
    response_.reply_text(
      status_code::NotFound,
      "<html><body>Not found</body><html>",
      "text/html"
    );
  } else {
    handler_(request_, &response_);
  }
  return true;
}

int Connection::get_response_status_code() const
{
  return response_.get_status_code();
}

std::string Connection::get_response_status_text() const
{
  return response_.get_status_text();
}

std::string Connection::get_protocol_version() const
{
  return response_.get_protocol_version();
}

std::string Connection::get_response_header(
  const std::string & header_name) const
{
  return response_.get_header(header_name);
}

Stream::UPtr Connection::extract_data_stream()
{
  // HEAD requests must not return body.
  if (request_.get_method() != request_type::kHead) {
    return MemoryStream::open_copy_of(response_.get_data_as_string());
  } else {
    // Return empty data stream for HEAD requests.
    return MemoryStream::open_copy_of(nullptr, 0);
  }
}

}  // namespace fake
}  // namespace http
}  // namespace rmf2_scheduler
