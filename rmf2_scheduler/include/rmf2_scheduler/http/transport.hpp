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

#ifndef RMF2_SCHEDULER__HTTP__TRANSPORT_HPP_
#define RMF2_SCHEDULER__HTTP__TRANSPORT_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rmf2_scheduler/http/connection.hpp"

namespace rmf2_scheduler
{

namespace http
{

class Transport
{
public:
  using HeaderList = std::vector<std::pair<std::string, std::string>>;

  Transport() = default;

  virtual ~Transport() {}

  virtual std::shared_ptr<Connection> create_connection(
    const std::string & url,
    const std::string & method,
    const HeaderList & headers,
    const std::string & user_agent,
    const std::string & referer,
    std::string & error
  ) = 0;

  static std::shared_ptr<Transport> create_default();

private:
  RS_DISABLE_COPY(Transport)
};

}  // namespace http
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__TRANSPORT_HPP_
