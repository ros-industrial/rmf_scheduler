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

#include "rmf2_scheduler/http/transport_curl.hpp"
#include "rmf2_scheduler/http/connection_curl.hpp"
#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler
{

namespace http
{

namespace curl
{

Transport::Transport(const std::shared_ptr<CURLInterface> & curl_interface)
: curl_interface_(curl_interface)
{
  // TODO(Briancbn): setup CA
  LOG_DEBUG("curl::Transport created");
}

Transport::Transport(
  const std::shared_ptr<CURLInterface> & curl_interface,
  const std::string & proxy
)
: curl_interface_{curl_interface}, proxy_{proxy}
{
  // TODO(Briancbn): setup CA
  LOG_DEBUG("curl::Transport created with proxy %s", proxy_.c_str());
}

Transport::~Transport()
{
  LOG_DEBUG("curl::Transport destroyed");
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
  CURL * curl_handle = curl_interface_->easy_init();

  if (!curl_handle) {
    LOG_ERROR("Failed to initialize CURL");
    error = "Transport create_connection failed, failed to initialize CURL";
    return connection;
  }

  LOG_DEBUG("Sending a %s request to ", method.c_str(), url.c_str());

  CURLcode code = curl_interface_->easy_setopt_str(curl_handle, CURLOPT_URL, url);

  // TODO(Briancbn): set CA

  if (code == CURLE_OK) {
    code =
      curl_interface_->easy_setopt_int(curl_handle, CURLOPT_SSL_VERIFYPEER, 1);
  }
  if (code == CURLE_OK) {
    code =
      curl_interface_->easy_setopt_int(curl_handle, CURLOPT_SSL_VERIFYHOST, 2);
  }
  if (code == CURLE_OK && !user_agent.empty()) {
    code = curl_interface_->easy_setopt_str(
      curl_handle, CURLOPT_USERAGENT,
      user_agent);
  }
  if (code == CURLE_OK && !referer.empty()) {
    code = curl_interface_->easy_setopt_str(curl_handle, CURLOPT_REFERER, referer);
  }

  if (code == CURLE_OK && !proxy_.empty()) {
    code = curl_interface_->easy_setopt_str(curl_handle, CURLOPT_PROXY, proxy_);
  }

  // TODO(Briancbn): set dns and more

  // Setup HTTP request method and optional request body.
  if (code == CURLE_OK) {
    if (method == request_type::kGet) {
      code = curl_interface_->easy_setopt_int(curl_handle, CURLOPT_HTTPGET, 1);
    } else if (method == request_type::kHead) {
      code = curl_interface_->easy_setopt_int(curl_handle, CURLOPT_NOBODY, 1);
    } else if (method == request_type::kPut) {
      code = curl_interface_->easy_setopt_int(curl_handle, CURLOPT_UPLOAD, 1);
    } else {
      // POST and custom request methods
      code = curl_interface_->easy_setopt_int(curl_handle, CURLOPT_POST, 1);
      if (code == CURLE_OK) {
        code = curl_interface_->easy_setopt_ptr(
          curl_handle, CURLOPT_POSTFIELDS,
          nullptr);
      }
      if (code == CURLE_OK && method != request_type::kPost) {
        code = curl_interface_->easy_setopt_str(
          curl_handle,
          CURLOPT_CUSTOMREQUEST, method);
      }
    }
  }

  if (code != CURLE_OK) {
    error = "Transport create_connection failed, curl error: " +
      curl_interface_->easy_strerror(code);
    curl_interface_->easy_cleanup(curl_handle);
    return connection;
  }

  connection = std::make_shared<http::curl::Connection>(
    curl_handle, method, curl_interface_
  );

  // send headers
  if (!connection->send_headers(headers, error)) {
    connection.reset();
  }

  return connection;
}

}  // namespace curl
}  // namespace http
}  // namespace rmf2_scheduler
