// Copyright 2023 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF_NOTIFICATION__WEBSOCKET_CLIENT_HPP_
#define RMF_NOTIFICATION__WEBSOCKET_CLIENT_HPP_

#include <memory>
#include <string>

#include "rmf_notification/notification_client_base.hpp"

namespace rmf_notification
{

class WebsocketClient : public NotificationClientBase
{
public:
  WebsocketClient();
  virtual ~WebsocketClient();
  void init(const std::string & endpoint_uri) override;

  bool is_connected() const override;

  void publish(
    const Message & message) override;

  void shutdown() override;

  class Impl;

private:
  std::unique_ptr<Impl> impl_ptr_;
};

}  // namespace rmf_notification

#endif  // RMF_NOTIFICATION__WEBSOCKET_CLIENT_HPP_
