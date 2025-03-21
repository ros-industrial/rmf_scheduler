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

#include "rmf2_scheduler/http/memory_stream.hpp"
#include "rmf2_scheduler/http/request.hpp"
#include "rmf2_scheduler/utils/string_utils.hpp"
#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler
{

namespace http
{

// REQUEST
Request::Request(
  const std::string & url,
  const std::string & method,
  std::shared_ptr<Transport> transport
)
: transport_(transport), request_url_(url), method_(method)
{
  LOG_DEBUG("http::Request created");
  if (!transport_) {
    transport_ = http::Transport::create_default();
  }
}

Request::~Request()
{
  LOG_DEBUG("http::Request destroyed");
}

void Request::set_accept(const std::string & accept_mime_types)
{
  if (!transport_) {
    LOG_ERROR("Request already sent");
    return;
  }
  accept_ = accept_mime_types;
}

const std::string & Request::get_accept() const
{
  return accept_;
}

void Request::set_content_type(const std::string & content_type)
{
  if (!transport_) {
    LOG_ERROR("Request already sent");
    return;
  }
  content_type_ = content_type;
}

const std::string & Request::get_content_type() const
{
  return content_type_;
}

void Request::add_header(const std::string & header, const std::string & value)
{
  if (!transport_) {
    LOG_ERROR("Request already sent");
    return;
  }
  headers_.emplace(header, value);
}

void Request::add_headers(const HeaderList & headers)
{
  if (!transport_) {
    LOG_ERROR("Request already sent");
    return;
  }
  headers_.insert(headers.begin(), headers.end());
}

bool Request::add_request_body(const void * data, size_t size, std::string & error)
{
  if (!_send_request_if_needed(error)) {
    return false;
  }

  // Create memory stream
  Stream::UPtr stream = MemoryStream::open_copy_of(data, size);
  return connection_->set_request_data(std::move(stream), error);
}

bool Request::add_request_body(const nlohmann::json & json, std::string & error)
{
  auto body_data = json.dump(4);
  return add_request_body(body_data.c_str(), body_data.size(), error);
}

bool Request::add_request_body(Stream::UPtr stream, std::string & error)
{
  return _send_request_if_needed(error) &&
         connection_->set_request_data(std::move(stream), error);
}

bool Request::add_response_stream(Stream::UPtr stream, std::string & error)
{
  if (!_send_request_if_needed(error)) {
    return false;
  }
  connection_->set_response_data(std::move(stream));
  return true;
}

const std::string & Request::get_request_url() const
{
  return request_url_;
}

const std::string & Request::get_request_method() const
{
  return method_;
}

void Request::set_referer(const std::string & referer)
{
  if (!transport_) {
    LOG_ERROR("Request already sent");
    return;
  }
  referer_ = referer;
}

const std::string & Request::get_referer() const
{
  return referer_;
}

void Request::set_user_agent(const std::string & user_agent)
{
  if (!transport_) {
    LOG_ERROR("Request already sent");
    return;
  }
  user_agent_ = user_agent;
}

const std::string & Request::get_user_agent() const
{
  return user_agent_;
}

void Request::add_range(int64_t bytes)
{
  if (!transport_) {
    LOG_ERROR("Request already sent");
    return;
  }
  if (bytes < 0) {
    ranges_.emplace_back(Request::range_value_omitted, -bytes);
  } else {
    ranges_.emplace_back(bytes, Request::range_value_omitted);
  }
}
void Request::add_range(uint64_t from_byte, uint64_t to_byte)
{
  if (!transport_) {
    LOG_ERROR("Request already sent");
    return;
  }
  ranges_.emplace_back(from_byte, to_byte);
}

std::unique_ptr<Response> Request::get_response_and_block(
  std::string & error)
{
  if (!_send_request_if_needed(error) || !connection_->perform_request(error)) {
    return std::unique_ptr<Response>();
  }
  std::unique_ptr<Response> response(new Response(connection_));
  connection_.reset();
  transport_.reset();  // Indicate that the response has been received
  return response;
}

bool Request::_send_request_if_needed(std::string & error)
{
  if (!transport_) {
    error = "HTTP response already received";

    return false;
  }

  if (connection_) {
    return true;
  }

  HeaderList headers;
  headers.reserve(headers_.size());
  for (auto & itr : headers_) {
    headers.emplace_back(itr.first, itr.second);
  }

  std::vector<std::string> ranges;
  if (method_ != request_type::kHead) {
    ranges.reserve(ranges_.size());
    for (auto p : ranges_) {
      if (p.first != range_value_omitted ||
        p.second != range_value_omitted)
      {
        std::string range;
        if (p.first != range_value_omitted) {
          range = std::to_string(p.first);
        }
        range += '-';
        if (p.second != range_value_omitted) {
          range += std::to_string(p.second);
        }
        ranges.push_back(range);
      }
    }
  }
  if (!ranges.empty()) {
    headers.emplace_back(
      request_header::kRange,
      "bytes=" + string_utils::join(",", ranges));
  }
  headers.emplace_back(request_header::kAccept, get_accept());
  if (method_ != request_type::kGet && method_ != request_type::kHead) {
    if (!content_type_.empty()) {
      headers.emplace_back(request_header::kContentType, content_type_);
    }
  }
  connection_ = transport_->create_connection(
    request_url_,
    method_,
    headers,
    user_agent_,
    referer_,
    error
  );

  return static_cast<bool>(connection_);
}

// RESPONSE
Response::Response(const std::shared_ptr<Connection> & connection)
: connection_{connection}
{
  LOG_DEBUG("http::Response created");
}

Response::~Response()
{
  LOG_DEBUG("http::Response destroyed");
}

bool Response::is_successful() const
{
  int code = get_status_code();
  return code >= status_code::Continue && code < status_code::BadRequest;
}

int Response::get_status_code() const
{
  if (!connection_) {
    return -1;
  }
  return connection_->get_response_status_code();
}

std::string Response::get_status_text() const
{
  if (!connection_) {
    return std::string();
  }
  return connection_->get_response_status_text();
}

std::string Response::get_content_type() const
{
  return get_header(response_header::kContentType);
}

Stream::UPtr Response::extract_data_stream()
{
  return connection_->extract_data_stream();
}

std::vector<uint8_t> Response::extract_data()
{
  std::vector<uint8_t> data;
  Stream::UPtr stream = connection_->extract_data_stream();
  if (stream) {
    data.resize(stream->remaining_size());
    stream->read(data.data(), data.size(), nullptr);
  }
  return data;
}

std::string Response::extract_data_as_string()
{
  std::vector<uint8_t> data = extract_data();
  return std::string{data.begin(), data.end()};
}

std::string Response::get_header(const std::string & header_name) const
{
  if (connection_) {
    return connection_->get_response_header(header_name);
  }
  return std::string();
}

}  // namespace http
}  // namespace rmf2_scheduler
