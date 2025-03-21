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

#include <cstring>

#include "rmf2_scheduler/http/connection_curl.hpp"
#include "rmf2_scheduler/http/memory_stream.hpp"
#include "rmf2_scheduler/log.hpp"
#include "rmf2_scheduler/utils/string_utils.hpp"

namespace rmf2_scheduler
{

namespace http
{

namespace curl
{

const size_t DEFAULT_CHUNK_SIZE = 1024;

static int curl_trace(
  CURL * /*handle*/,
  curl_infotype type,
  char * data,
  size_t size,
  void * /*userp*/
)
{
  std::string msg(data, size);
  switch (type) {
    case CURLINFO_TEXT:
      LOG_DEBUG("== Info: %s", msg.c_str());
      break;
    case CURLINFO_HEADER_OUT:
      LOG_DEBUG("=> Send headers:\n%s", msg.c_str());
      break;
    case CURLINFO_DATA_OUT:
      LOG_DEBUG("=> Send data:\n%s", msg.c_str());
      break;
    case CURLINFO_SSL_DATA_OUT:
      LOG_DEBUG("=> Send SSL data", msg.c_str());
      break;
    case CURLINFO_HEADER_IN:
      LOG_DEBUG("<= Recv header: %s", msg.c_str());
      break;
    case CURLINFO_DATA_IN:
      LOG_DEBUG("<= Recv data:\n%s", msg.c_str());
      break;
    case CURLINFO_SSL_DATA_IN:
      LOG_DEBUG("<= Recv SSL data", msg.c_str());
      break;
    default:
      break;
  }
  return 0;
}

Connection::Connection(
  CURL * curl_handle,
  const std::string & method,
  const std::shared_ptr<CURLInterface> & curl_interface
)
: curl_handle_(curl_handle),
  method_(method),
  curl_interface_(curl_interface)
{
  // Store the connection pointer inside the CURL handle so we can easily
  // retrieve it when doing asynchronous I/O.
  curl_interface_->easy_setopt_ptr(curl_handle_, CURLOPT_PRIVATE, this);
  LOG_DEBUG("curl::Connection created: %s", method_.c_str());
}

Connection::~Connection()
{
  if (header_list_) {
    curl_slist_free_all(header_list_);
  }
  curl_interface_->easy_cleanup(curl_handle_);
  LOG_DEBUG("curl::Connection destroyed");
}

bool Connection::send_headers(
  const HeaderList & headers,
  std::string & /* error */
)
{
  request_headers_.insert(headers.begin(), headers.end());
  return true;
}
bool Connection::set_request_data(
  Stream::UPtr data,
  std::string & /*error*/
)
{
  request_buffer_ = std::move(data);
  return true;
}

bool Connection::set_response_data(Stream::UPtr data)
{
  response_buffer_ = std::move(data);
  return true;
}

void Connection::prepare_request()
{
  // Set debug and verbosity
  if (log::getLogLevel() <= LogLevel::DEBUG) {
    curl_interface_->easy_setopt_callback(curl_handle_, CURLOPT_DEBUGFUNCTION, &curl_trace);
    curl_interface_->easy_setopt_int(curl_handle_, CURLOPT_VERBOSE, 1);
  }

  if (method_ != request_type::kGet) {
    size_t data_size = 0;
    if (request_buffer_) {
      // Data size is known (either no data, or data size is available).
      data_size = request_buffer_->remaining_size();
    }

    if (method_ == request_type::kPut) {
      curl_interface_->easy_setopt_off_t(
        curl_handle_,
        CURLOPT_INFILESIZE_LARGE,
        data_size
      );
    } else {
      curl_interface_->easy_setopt_off_t(
        curl_handle_,
        CURLOPT_POSTFIELDSIZE_LARGE,
        data_size
      );
    }

    // TODO(Briancbn): Data size is unknown, so use chunked upload.
  }

  if (request_buffer_) {
    // Setup request callback and data
    curl_interface_->easy_setopt_callback(
      curl_handle_,
      CURLOPT_READFUNCTION,
      &Connection::read_callback
    );
    curl_interface_->easy_setopt_ptr(
      curl_handle_,
      CURLOPT_READDATA,
      this
    );
  }

  // Setup headers
  if (!request_headers_.empty()) {
    for (const auto & pair : request_headers_) {
      std::string header = string_utils::join(": ", pair.first, pair.second);
      LOG_DEBUG("Request headers: %s", header.c_str());
      header_list_ = curl_slist_append(header_list_, header.c_str());
    }
    curl_interface_->easy_setopt_ptr(curl_handle_, CURLOPT_HTTPHEADER, header_list_);
  }

  // Setup response callback and data
  if (!response_buffer_) {
    response_buffer_ = std::make_unique<MemoryStream>(DEFAULT_CHUNK_SIZE);
  }
  if (method_ != request_type::kHead) {
    curl_interface_->easy_setopt_callback(
      curl_handle_,
      CURLOPT_WRITEFUNCTION,
      &Connection::write_callback
    );
    curl_interface_->easy_setopt_ptr(
      curl_handle_,
      CURLOPT_WRITEDATA,
      this
    );
  }

  // Setup response headers
  curl_interface_->easy_setopt_callback(
    curl_handle_,
    CURLOPT_HEADERFUNCTION,
    &Connection::header_callback
  );
  curl_interface_->easy_setopt_ptr(
    curl_handle_,
    CURLOPT_HEADERDATA,
    this
  );
}

bool Connection::perform_request(std::string & error_text)
{
  prepare_request();
  CURLcode ret = curl_interface_->easy_perform(curl_handle_);
  if (ret != CURLE_OK) {
    error_text = curl_interface_->easy_strerror(ret);
  } else {
    LOG_DEBUG(
      "Response: %d (%s)",
      get_response_status_code(),
      get_response_status_text().c_str()
    );
  }

  return ret == CURLE_OK;
}

int Connection::get_response_status_code() const
{
  int status_code = 0;
  curl_interface_->easy_getinfo_int(
    curl_handle_,
    CURLINFO_RESPONSE_CODE,
    &status_code
  );
  return status_code;
}

std::string Connection::get_response_status_text() const
{
  return status_text_;
}

std::string Connection::get_protocol_version() const
{
  return protocol_version_;
}

std::string Connection::get_response_header(
  const std::string & header_name
) const
{
  auto p = response_headers_.find(string_utils::to_lower_ascii(header_name));
  return p != response_headers_.end() ? p->second : "";
}

Stream::UPtr Connection::extract_data_stream()
{
  response_buffer_->set_position(0);
  return std::move(response_buffer_);
}

size_t Connection::write_callback(
  char * ptr,
  size_t size,
  size_t num,
  void * data
)
{
  Connection * me = reinterpret_cast<Connection *>(data);
  size_t data_len = size * num;
  size_t write_size;

  me->response_buffer_->write(ptr, data_len, &write_size);

  LOG_DEBUG("Receiving data: %s", std::string{ptr, write_size}.c_str());
  return write_size;
}

size_t Connection::read_callback(
  char * ptr,
  size_t size,
  size_t num,
  void * data
)
{
  Connection * me = reinterpret_cast<Connection *>(data);
  size_t data_len = size * num;
  size_t read_size = 0;
  bool success = me->request_buffer_->read(ptr, data_len, &read_size);
  if (success) {
    LOG_DEBUG("Sending data: %s", std::string{ptr, read_size}.c_str());
  }
  return success ? read_size : CURL_READFUNC_ABORT;
}

size_t Connection::header_callback(
  char * ptr,
  size_t size,
  size_t num,
  void * data
)
{
  Connection * me = reinterpret_cast<Connection *>(data);
  size_t header_len = size * num;
  std::string header(ptr, header_len);

  // Remove newlines at the end of header line.
  while (!header.empty() && (header.back() == '\r' || header.back() == '\n')) {
    header.pop_back();
  }

  LOG_DEBUG("Response header: %s", header.c_str());

  if (!me->status_text_set_) {
    // First header - response code as  "HTTP/1.1 200 OK".
    // Need to extract the OK part
    auto pair = string_utils::split_at_first(header, " ");
    me->protocol_version_ = pair.first;
    me->status_text_ = string_utils::split_at_first(pair.second, " ").second;
    me->status_text_set_ = true;
  } else {
    auto pair = string_utils::split_at_first(header, ":");
    if (!pair.second.empty()) {
      // According to https://www.w3.org/Protocols/rfc2616/rfc2616-sec4.html.
      // Headers are case-insensitive. Convert all headers to lower case for
      // easy processing.
      pair.first = string_utils::to_lower_ascii(pair.first);
      me->response_headers_.insert(pair);
    }
  }
  return header_len;
}


}  // namespace curl
}  // namespace http
}  // namespace rmf2_scheduler
