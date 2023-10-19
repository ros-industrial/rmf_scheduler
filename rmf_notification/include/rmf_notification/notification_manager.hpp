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
#include <mutex>
#include <string>

#include "rmf_notification/message.hpp"
#include "rmf_notification/notification_client_base.hpp"

namespace rmf_notification
{

class NotificationManager
{
public:
  /// Method to get singleton instance of notification manager
  /**
   * This method is threadsafe. It has mutex lock which allows only one
   * thread to access this method at the same time frame
   *
   * \return[out] Shared pointer instance of notification manager
  */
  static std::shared_ptr<NotificationManager> get();

  /// Method to publish notifications via both websocket and telegram
  /**
   * This method is asynchronous type as the message to be sent is stored in a
   * local cache and sent over by separate threads
   *
   * \param[in] payload message to be sent via notification system
   * \param[in] type categories for different notifications
   * \return[out] Id assigned to the messages to be sent
  */
  std::string publish(
    const std::string payload,
    const std::string type);

  /// Method to check connection status with both websocket and telegram modules
  /**
   * For websocket live connection tracking is performed which means the connection status
   * changes to disconnected on immediate disconnection. For telegram connection status
   * is updated only when a new request is triggered
   *
   * \return[out] Vector specifying the endpoint uri and it status
  */
  std::unordered_map<std::string, bool> get_connections() const;

  /// Method to create a client to a specified uri
  /**
   * This method is NOT thread safe
   */
  template<typename ClientBaseT, typename ... ArgsT>
  std::shared_ptr<ClientBaseT> create_client(
    const std::string & uri,
    ArgsT &&... args);

  virtual ~NotificationManager();

private:
  // unique pointer to store instance for mobile notification manager
  std::unordered_map<std::string, std::shared_ptr<NotificationClientBase>> clients_;

  // shared pointer to store singleton instance for notification manager
  static std::shared_ptr<NotificationManager> instance_;
  // mutex lock which is used when acquiring instance of notification manager object
  static std::mutex m_instance_;

  /// Constructor
  NotificationManager();
  // Not copiable
  NotificationManager(const NotificationManager &) = delete;
  NotificationManager & operator=(const NotificationManager &) = delete;
};

template<typename ClientBaseT, typename ... ArgsT>
std::shared_ptr<ClientBaseT> NotificationManager::create_client(
  const std::string & uri,
  ArgsT &&... args)
{
  static_assert(
    std::is_base_of_v<NotificationClientBase, ClientBaseT>,
    "Notification Client must be derived from NotificationClientBase"
  );

  // use a local variable to mimic the constructor
  auto ptr = std::make_shared<ClientBaseT>(std::forward<ArgsT>(args)...);

  // Initialize the client
  ptr->init(uri);

  clients_.emplace(
    uri,
    std::static_pointer_cast<ClientBaseT>(ptr));

  return ptr;
}

}  // namespace rmf_notification

#endif  // RMF_NOTIFICATION__NOTIFICATION_MANAGER_HPP_
