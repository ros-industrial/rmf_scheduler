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

#ifndef RMF_NOTIFICATION__MESSAGE_HPP_
#define RMF_NOTIFICATION__MESSAGE_HPP_

#include <string>
#include <unordered_map>

namespace rmf_notification
{

enum ConnectionState { CONNECTED, DISCONNECTED };
enum NotificationType { WEBSOCKET, REST };

struct Message
{
  explicit Message(
    std::string _type,
    std::string _payload,
    uint64_t _timestamp
  );

  Message() = default;

  /// Id for message
  std::string message_id;

  /// Type or topic for message
  std::string type;

  /// Actual payload for the message
  std::string payload;

  /// Timestamp of message
  uint64_t timestamp;
};

}  // namespace rmf_notification

#endif  // RMF_NOTIFICATION__MESSAGE_HPP_
