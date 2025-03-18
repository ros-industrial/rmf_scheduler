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

#include "rmf2_scheduler/http/request.hpp"
#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler
{

namespace http
{

// request_type
const char request_type::kOptions[] = "OPTIONS";
const char request_type::kGet[] = "GET";
const char request_type::kHead[] = "HEAD";
const char request_type::kPost[] = "POST";
const char request_type::kPut[] = "PUT";
const char request_type::kPatch[] = "PATCH";
const char request_type::kDelete[] = "DELETE";
const char request_type::kTrace[] = "TRACE";
const char request_type::kConnect[] = "CONNECT";
const char request_type::kCopy[] = "COPY";
const char request_type::kMove[] = "MOVE";

// request_header
const char request_header::kAccept[] = "Accept";
const char request_header::kAcceptCharset[] = "Accept-Charset";
const char request_header::kAcceptEncoding[] = "Accept-Encoding";
const char request_header::kAcceptLanguage[] = "Accept-Language";
const char request_header::kAllow[] = "Allow";
const char request_header::kAuthorization[] = "Authorization";
const char request_header::kCacheControl[] = "Cache-Control";
const char request_header::kConnection[] = "Connection";
const char request_header::kContentEncoding[] = "Content-Encoding";
const char request_header::kContentLanguage[] = "Content-Language";
const char request_header::kContentLength[] = "Content-Length";
const char request_header::kContentLocation[] = "Content-Location";
const char request_header::kContentMd5[] = "Content-MD5";
const char request_header::kContentRange[] = "Content-Range";
const char request_header::kContentType[] = "Content-Type";
const char request_header::kCookie[] = "Cookie";
const char request_header::kDate[] = "Date";
const char request_header::kExpect[] = "Expect";
const char request_header::kExpires[] = "Expires";
const char request_header::kFrom[] = "From";
const char request_header::kHost[] = "Host";
const char request_header::kIfMatch[] = "If-Match";
const char request_header::kIfModifiedSince[] = "If-Modified-Since";
const char request_header::kIfNoneMatch[] = "If-None-Match";
const char request_header::kIfRange[] = "If-Range";
const char request_header::kIfUnmodifiedSince[] = "If-Unmodified-Since";
const char request_header::kLastModified[] = "Last-Modified";
const char request_header::kMaxForwards[] = "Max-Forwards";
const char request_header::kPragma[] = "Pragma";
const char request_header::kProxyAuthorization[] = "Proxy-Authorization";
const char request_header::kRange[] = "Range";
const char request_header::kReferer[] = "Referer";
const char request_header::kTE[] = "TE";
const char request_header::kTrailer[] = "Trailer";
const char request_header::kTransferEncoding[] = "Transfer-Encoding";
const char request_header::kUpgrade[] = "Upgrade";
const char request_header::kUserAgent[] = "User-Agent";
const char request_header::kVia[] = "Via";
const char request_header::kWarning[] = "Warning";

// response_header
const char response_header::kAcceptRanges[] = "Accept-Ranges";
const char response_header::kAge[] = "Age";
const char response_header::kAllow[] = "Allow";
const char response_header::kCacheControl[] = "Cache-Control";
const char response_header::kConnection[] = "Connection";
const char response_header::kContentEncoding[] = "Content-Encoding";
const char response_header::kContentLanguage[] = "Content-Language";
const char response_header::kContentLength[] = "Content-Length";
const char response_header::kContentLocation[] = "Content-Location";
const char response_header::kContentMd5[] = "Content-MD5";
const char response_header::kContentRange[] = "Content-Range";
const char response_header::kContentType[] = "Content-Type";
const char response_header::kDate[] = "Date";
const char response_header::kETag[] = "ETag";
const char response_header::kExpires[] = "Expires";
const char response_header::kLastModified[] = "Last-Modified";
const char response_header::kLocation[] = "Location";
const char response_header::kPragma[] = "Pragma";
const char response_header::kProxyAuthenticate[] = "Proxy-Authenticate";
const char response_header::kRetryAfter[] = "Retry-After";
const char response_header::kServer[] = "Server";
const char response_header::kSetCookie[] = "Set-Cookie";
const char response_header::kTrailer[] = "Trailer";
const char response_header::kTransferEncoding[] = "Transfer-Encoding";
const char response_header::kUpgrade[] = "Upgrade";
const char response_header::kVary[] = "Vary";
const char response_header::kVia[] = "Via";
const char response_header::kWarning[] = "Warning";
const char response_header::kWwwAuthenticate[] = "WWW-Authenticate";


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

bool Request::add_request_body(const std::string & stream, std::string & error)
{
  if (!_send_request_if_needed(error)) {
    return false;
  }
  return connection_->set_request_data(stream, error);
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
  for (auto & itr : headers) {
    headers.emplace_back(itr.first, itr.second);
  }
  // std::vector<std::string> ranges;
  // if (method_ != request_type::kHead) {
  //   ranges.reserve(ranges_.size());
  //   for (auto p : ranges_) {
  //     if (p.first != range_value_omitted ||
  //         p.second != range_value_omitted) {
  //       std::string range;
  //       if (p.first != range_value_omitted) {
  //         range = brillo::string_utils::ToString(p.first);
  //       }
  //       range += '-';
  //       if (p.second != range_value_omitted) {
  //         range += brillo::string_utils::ToString(p.second);
  //       }
  //       ranges.push_back(range);
  //     }
  //   }
  // }
  // if (!ranges.empty()) {
  //   headers.emplace_back(
  //       request_header::kRange,
  //       "bytes=" + brillo::string_utils::Join(",", ranges));
  // }
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

std::string Response::extract_data_stream()
{
  return connection_->extract_data_stream();
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
