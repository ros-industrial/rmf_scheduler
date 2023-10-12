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

#ifndef RMF_NOTIFICATION__NOTIFICATION_MANAGER_HPP_
#define RMF_NOTIFICATION__NOTIFICATION_MANAGER_HPP_

#include <unordered_map>
#include <utility>
#include <vector>
#include <memory>
#include <string>

#include "rmf_notification/message.hpp"

namespace rmf_notification
{

class NotificationManager
{
public:
  NotificationManager(
    std::string websocket_uri,
    std::string mobile_uri);
  std::string publish_state(
    const std::string payload,
    const std::string type);
  std::vector<std::pair<NotificationType, ConnectionState>> check_connection_state();
  void clear_messages();
  ~NotificationManager();

private:
  class WebsocketImplementation;
  class MobileImplementation;

  std::string websocket_uri_;
  std::string mobile_uri_;
  std::unique_ptr<WebsocketImplementation> websocket_pimpl_;
  std::unique_ptr<MobileImplementation> mobile_pimpl_;
  std::unordered_map<std::string, Message> message_map_;
};

}  // namespace rmf_notification

#endif  // RMF_NOTIFICATION__NOTIFICATION_MANAGER_HPP_
