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

#ifndef RMF2_SCHEDULER__HTTP__TRANSPORT_CURL_HPP_
#define RMF2_SCHEDULER__HTTP__TRANSPORT_CURL_HPP_

#include <memory>
#include <string>

#include "rmf2_scheduler/http/transport.hpp"
#include "rmf2_scheduler/http/curl_api.hpp"

namespace rmf2_scheduler
{

namespace http
{

namespace curl
{

class Transport : public http::Transport
{
public:
  explicit Transport(const std::shared_ptr<CURLInterface> & curl_interface);

  /// Creates a transport object using a proxy.
  /**
   * |proxy| is of the form [protocol://][user:password@]host[:port].
   * If not defined, protocol is assumed to be http://.
   */
  Transport(
    const std::shared_ptr<CURLInterface> & curl_interface,
    const std::string & proxy
  );

  virtual ~Transport();

  std::shared_ptr<http::Connection> create_connection(
    const std::string & url,
    const std::string & method,
    const HeaderList & headers,
    const std::string & user_agent,
    const std::string & referer,
    std::string & error
  ) override;

private:
  RS_DISABLE_COPY(Transport)

  std::shared_ptr<CURLInterface> curl_interface_;
  std::string proxy_;
};

}  // namespace curl
}  // namespace http
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__TRANSPORT_CURL_HPP_
