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

#include <condition_variable>
#include <thread>

#include "cpprest/http_client.h"
#include "nlohmann/json.hpp"

#include "websocketpp/config/asio_client.hpp"
#include "websocketpp/client.hpp"

#include "rmf_scheduler/utils/uuid.hpp"
#include "rmf_scheduler/log.hpp"
#include "rmf_notification/notification_manager.hpp"

namespace rmf_notification
{

std::shared_ptr<NotificationManager> NotificationManager::instance_(nullptr);
std::mutex NotificationManager::m_instance_;

std::shared_ptr<NotificationManager> NotificationManager::get()
{
  std::lock_guard<std::mutex> lock(m_instance_);
  if (!instance_) {
    std::shared_ptr<NotificationManager> temp_instance(
      new NotificationManager());
    instance_.swap(temp_instance);
  }
  return instance_;
}

NotificationManager::NotificationManager()
{
}

NotificationManager::~NotificationManager()
{
}

std::string NotificationManager::publish(
  const std::string payload,
  const std::string type)
{
  Message message;
  message.payload = payload;
  message.type = type;
  message.message_id = rmf_scheduler::utils::gen_uuid();
  for (auto & client : clients_) {
    client.second->publish(message);
  }
  return message.message_id;
}

std::unordered_map<std::string, bool> NotificationManager::get_connections() const
{
  std::unordered_map<std::string, bool> results;
  for (auto & client : clients_) {
    results[client.first] = client.second->is_connected();
  }
  return results;
}

}  // namespace rmf_notification
