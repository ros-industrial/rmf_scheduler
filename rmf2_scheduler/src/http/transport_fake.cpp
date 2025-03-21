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
#include "rmf2_scheduler/http/memory_stream.hpp"
#include "rmf2_scheduler/http/transport_fake.hpp"
#include "rmf2_scheduler/utils/url_utils.hpp"
#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler
{

namespace http
{

namespace fake
{

namespace
{
std::string get_handler_map_key(
  const std::string & url,
  const std::string & method
)
{
  return method + ":" + url;
}
}


// Transport
Transport::Transport()
{
  LOG_DEBUG("fake::Transport created.");
}

Transport::~Transport()
{
  LOG_DEBUG("fake::Transport destroyed.");
}

std::shared_ptr<http::Connection> Transport::create_connection(
  const std::string & url,
  const std::string & method,
  const HeaderList & headers,
  const std::string & user_agent,
  const std::string & referer,
  std::string & error
)
{
  std::shared_ptr<http::Connection> connection;
  if (!create_connection_error_.empty()) {
    error = create_connection_error_;
    return connection;
  }

  HeaderList headers_copy = headers;
  if (!user_agent.empty()) {
    headers_copy.push_back(
      std::make_pair(http::request_header::kUserAgent, user_agent)
    );
  }

  if (!referer.empty()) {
    headers_copy.push_back(
      std::make_pair(http::request_header::kReferer, referer)
    );
  }

  // Create fake connection
  auto handler = get_handler(
    url::remove_query_string(url, true),
    method
  );
  connection = std::make_shared<http::fake::Connection>(
    url, method, handler
  );

  if (!connection) {
    error = "Unable to create Connection object";
  }

  if (!connection->send_headers(headers_copy, error)) {
    connection.reset();
  }
  request_count_++;
  return connection;
}

void Transport::add_handler(
  const std::string & url,
  const std::string & method,
  const HandlerCallback & handler
)
{
  // Make sure we can override/replace existing handlers.
  handlers_[get_handler_map_key(url, method)] = handler;
}

void Transport::add_simple_reply_handler(
  const std::string & url,
  const std::string & method,
  int status_code,
  const std::string & reply_text,
  const std::string & mime_type
)
{
  auto handler = [](
    int status_code,
    const std::string & reply_text,
    const std::string & mime_type,
    const ServerRequest & /* request */,
    ServerResponse * response
    ) {
      response->reply_text(status_code, reply_text, mime_type);
    };

  add_handler(
    url, method,
    std::bind(
      handler,
      status_code, reply_text, mime_type,
      std::placeholders::_1, std::placeholders::_2
    )
  );
}

HandlerCallback Transport::get_handler(
  const std::string & url,
  const std::string & method
) const
{
  // First try the exact combination of URL/Method
  auto p = handlers_.find(get_handler_map_key(url, method));
  if (p != handlers_.end()) {
    return p->second;
  }
  // If not found, try URL/*
  p = handlers_.find(get_handler_map_key(url, "*"));
  if (p != handlers_.end()) {
    return p->second;
  }
  // If still not found, try */method
  p = handlers_.find(get_handler_map_key("*", method));
  if (p != handlers_.end()) {
    return p->second;
  }
  // Finally, try */*
  p = handlers_.find(get_handler_map_key("*", "*"));
  return (p != handlers_.end()) ? p->second : HandlerCallback();
}


// ServerRequestResponseBase
ServerRequestResponseBase::ServerRequestResponseBase()
{
}

ServerRequestResponseBase::~ServerRequestResponseBase()
{
}

void ServerRequestResponseBase::set_data(Stream::UPtr stream)
{
  data_.resize(stream->remaining_size());
  stream->read(data_.data(), data_.size(), nullptr);
}

std::string ServerRequestResponseBase::get_data_as_string() const
{
  if (data_.empty()) {
    return std::string();
  }
  auto chars = reinterpret_cast<const char *>(data_.data());
  return std::string(chars, data_.size());
}

std::optional<nlohmann::json> ServerRequestResponseBase::get_data_as_json() const
{
  // TODO(anyone): check header include json mime
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(get_data_as_string());
  } catch (const std::exception & /*e*/) {
    return std::nullopt;
  }

  if (j.is_null()) {
    return std::nullopt;
  }

  return j;
}

std::string ServerRequestResponseBase::get_data_as_normalized_json_string() const
{
  std::string result;
  auto j = get_data_as_json();
  if (j.has_value()) {
    result = j->dump();
  }
  return result;
}

void ServerRequestResponseBase::add_headers(const HeaderList & headers)
{
  for (const auto & pair : headers) {
    if (pair.second.empty()) {
      headers_.erase(pair.first);
    } else {
      headers_.insert(pair);
    }
  }
}

std::string ServerRequestResponseBase::get_header(
  const std::string & header_name
) const
{
  auto p = headers_.find(header_name);
  return p != headers_.end() ? p->second : std::string();
}

// ServerRequest
ServerRequest::ServerRequest(
  const std::string & url,
  const std::string & method
)
: method_(method)
{
  auto params = url::get_query_string_parameters(url);
  url_ = url::remove_query_string(url, true);
  form_fields_.insert(params.begin(), params.end());
}

ServerRequest::~ServerRequest()
{
}

std::string ServerRequest::get_form_field(const std::string & field_name) const
{
  if (!form_fields_parsed_) {
    // TODO(anyone): handle mime
  }
  auto p = form_fields_.find(field_name);
  return p != form_fields_.end() ? p->second : std::string();
}

// Server Response
ServerResponse::ServerResponse()
{
}

ServerResponse::~ServerResponse()
{
}

void ServerResponse::reply(
  int status_code,
  const void * data,
  size_t data_size,
  const std::string & mime_type
)
{
  data_.clear();
  status_code_ = status_code;
  set_data(MemoryStream::open_copy_of(data, data_size));

  add_headers(
        {
          {response_header::kContentLength, std::to_string(data_size)},
          {response_header::kContentType, mime_type}
        });
}

void ServerResponse::reply_text(
  int status_code,
  const std::string & text,
  const std::string & mime_type
)
{
  reply(status_code, text.data(), text.size(), mime_type);
}

void ServerResponse::reply_json(
  int status_code,
  const nlohmann::json & json,
  const std::string & mime_type
)
{
  reply_text(status_code, json.dump(2), mime_type);
}

std::string ServerResponse::get_status_text() const
{
  static std::vector<std::pair<int, const char *>> status_text_map = {
    {100, "Continue"},
    {101, "Switching Protocols"},
    {102, "Processing"},
    {200, "OK"},
    {201, "Created"},
    {202, "Accepted"},
    {203, "Non-Authoritative Information"},
    {204, "No Content"},
    {205, "Reset Content"},
    {206, "Partial Content"},
    {207, "Multi-Status"},
    {208, "Already Reported"},
    {226, "IM Used"},
    {300, "Multiple Choices"},
    {301, "Moved Permanently"},
    {302, "Found"},
    {303, "See Other"},
    {304, "Not Modified"},
    {305, "Use Proxy"},
    {306, "Switch Proxy"},
    {307, "Temporary Redirect"},
    {308, "Permanent Redirect"},
    {400, "Bad Request"},
    {401, "Unauthorized"},
    {402, "Payment Required"},
    {403, "Forbidden"},
    {404, "Not Found"},
    {405, "Method Not Allowed"},
    {406, "Not Acceptable"},
    {407, "Proxy Authentication Required"},
    {408, "Request Timeout"},
    {409, "Conflict"},
    {410, "Gone"},
    {411, "Length Required"},
    {412, "Precondition Failed"},
    {413, "Request Entity Too Large"},
    {414, "Request - URI Too Long"},
    {415, "Unsupported Media Type"},
    {429, "Too Many Requests"},
    {431, "Request Header Fields Too Large"},
    {500, "Internal Server Error"},
    {501, "Not Implemented"},
    {502, "Bad Gateway"},
    {503, "Service Unavailable"},
    {504, "Gateway Timeout"},
    {505, "HTTP Version Not Supported"},
  };
  for (const auto & pair : status_text_map) {
    if (pair.first == status_code_) {
      return pair.second;
    }
  }
  return std::string();
}

void ServerRequestHandler::perform(
  const ServerRequest & request, ServerResponse * response
)
{
  if (call_count_ >= handler_callbacks_.size()) {
    LOG_ERROR(
      "Max handle callback [%lu] already reached!!!",
      handler_callbacks_.size()
    );
    return;
  }

  // Run the callback
  auto callback = handler_callbacks_[call_count_];
  callback(request, response);
  call_count_++;
}

}  // namespace fake
}  // namespace http
}  // namespace rmf2_scheduler
