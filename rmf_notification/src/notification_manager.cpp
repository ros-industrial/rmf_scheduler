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
#include <mutex>
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

using http_request = web::http::http_request;
using http_response = web::http::http_response;
using http_client = web::http::client::http_client;

using asio_client = websocketpp::config::asio_client;

class NotificationManager::WebsocketImplementation
{
public:
  WebsocketImplementation(
    std::string & websocket_uri,
    std::unordered_map<std::string, Message> & message_map)
  : websocket_uri_(websocket_uri), message_map_(message_map)
  {
    websocket_client_ptr_ = std::make_unique<websocketpp::client<asio_client>>();
    websocket_client_ptr_->clear_access_channels(websocketpp::log::alevel::all);
    websocket_client_ptr_->clear_error_channels(websocketpp::log::elevel::all);

    websocket_client_ptr_->init_asio();
    websocket_client_ptr_->start_perpetual();

    // websocket handler callback to be used when new connection is open
    websocket_client_ptr_->set_open_handler(
      [this](
        websocketpp::connection_hdl hdl)
      {
        RS_LOG_INFO("websocket connection is open");
        hdl_ptr_ = hdl.lock();
        connected_ = true;
      });

    // websocket handler callback to be used when existing connection is closed
    websocket_client_ptr_->set_close_handler(
      [this](
        websocketpp::connection_hdl)
      {
        RS_LOG_INFO("websocket connection is closed");
        connected_ = false;
      });

    // websocket handler callback to be used when connection request is failed
    websocket_client_ptr_->set_fail_handler(
      [this](
        websocketpp::connection_hdl)
      {
        RS_LOG_INFO("websocket connection request failed");
        connected_ = false;
      });

    websocketpp::lib::error_code ec_for_new;
    websocket_client_ptr_->connect(
      websocket_client_ptr_->get_connection(websocket_uri_, ec_for_new));
    if (ec_for_new) {
      RS_LOG_ERROR(
        "new websocket connection failed for reason: %s",
        ec_for_new.message().c_str());
    }

    // new thread for handling websocket client connections
    client_thread_ = std::thread(
      [this]()
      {
        RS_LOG_INFO("starting websocket run inside client thread");
        websocket_client_ptr_->run();
      });

    // new thread for background processing of messages to be sent via websocket client
    processing_thread_ = std::thread(
      [this]()
      {
        RS_LOG_INFO("starting websocket background runner in processing thread");
        runner();
      });
  }

  void publish_state(std::string & message_id)
  {
    RS_LOG_INFO(
      "websocket notification request raised for message with id: %s",
      message_id.c_str());
    std::unique_lock<std::mutex> lock(notify_mtx_);
    message_process_queue_.push(message_id);
    notify_trigger_.notify_one();
  }

  bool check_connection_state()
  {
    return connected_;
  }

  ~WebsocketImplementation()
  {
    shutdown_ = true;
    // triggering continuation of background runner for starting shutdown process
    notify_trigger_.notify_one();
    // closing websocket connection
    websocket_client_ptr_->close(
      hdl_ptr_,
      websocketpp::close::status::going_away,
      "");
    // stopping all websocket process
    websocket_client_ptr_->stop();

    // waiting for processing thread to complete its run
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    // waiting for client thread to complete its run
    if (client_thread_.joinable()) {
      client_thread_.join();
    }
  }

private:
  std::string & websocket_uri_;
  std::unordered_map<std::string, Message> & message_map_;
  std::unique_ptr<websocketpp::client<asio_client>> websocket_client_ptr_;
  std::weak_ptr<void> hdl_ptr_;
  std::queue<std::string> message_process_queue_;
  std::mutex notify_mtx_;
  std::condition_variable notify_trigger_;
  std::thread processing_thread_;
  std::thread client_thread_;
  bool connected_;
  bool shutdown_;

  void runner()
  {
    while (!shutdown_) {
      // acquiring lock for notification
      std::unique_lock<std::mutex> lock(notify_mtx_);

      // trying to reconnect if disconnected
      if (!connected_) {
        websocketpp::lib::error_code ec_for_reconnection;
        websocket_client_ptr_->connect(
          websocket_client_ptr_->get_connection(
            websocket_uri_,
            ec_for_reconnection
        ));

        if (ec_for_reconnection) {
          RS_LOG_INFO(
            "reconnecting websocket failed for reason: %s",
            ec_for_reconnection.message().c_str());
        }
      }

      // looping through the message process queue for sending them over websocket
      while (!message_process_queue_.empty() && connected_) {
        std::string message_id = message_process_queue_.front();
        auto message = message_map_.at(message_id);

        nlohmann::json msg;
        msg["type"] = message.type;
        msg["payload"] = message.payload;
        msg["timestamp"] = message.timestamp;
        msg["message_id"] = message.message_id;

        websocketpp::lib::error_code ec_for_sending;
        websocket_client_ptr_->send(
          hdl_ptr_,
          msg.dump(),
          websocketpp::frame::opcode::text,
          ec_for_sending
        );

        if (ec_for_sending) {
          RS_LOG_ERROR(
            "unable to publish websocket message: %s",
            ec_for_sending.message().c_str());
        } else {
          RS_LOG_INFO(
            "websocket broacasted message with id: %s",
            message.message_id.c_str());
          message_process_queue_.pop();
        }
      }

      // releasing the lock and waiting for notify from other methods
      notify_trigger_.wait(lock);
    }
  }
};

class NotificationManager::MobileImplementation
{
public:
  MobileImplementation(
    std::string & mobile_uri,
    std::unordered_map<std::string, Message> & message_map)
  :  message_map_(message_map)
  {
    // creating http client instance
    http_client_ptr_ = std::make_shared<http_client>(U(mobile_uri));
    // starting new thread for handling request process in background
    processing_thread_ = std::thread(
      [this]()
      {
        RS_LOG_INFO(
          "starting mobile notification background runner in processing thread");
        runner();
      });
  }

  void publish_state(std::string & message_id)
  {
    RS_LOG_INFO(
      "mobile notification request raised for message with id: %s",
      message_id.c_str());
    std::unique_lock<std::mutex> lock(notify_mtx_);
    message_process_queue_.push(message_id);
    notify_trigger_.notify_one();
  }

  bool check_connection_state()
  {
    return connected_;
  }

  ~MobileImplementation()
  {
    shutdown_ = true;
    notify_trigger_.notify_one();
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
  }

private:
  std::unordered_map<std::string, Message> & message_map_;
  std::shared_ptr<http_client> http_client_ptr_;
  std::queue<std::string> message_process_queue_;
  std::mutex notify_mtx_;
  std::condition_variable notify_trigger_;
  std::thread processing_thread_;
  bool connected_;
  bool shutdown_;

  void runner()
  {
    while (!shutdown_) {
      std::unique_lock<std::mutex> lock(notify_mtx_);

      while (!message_process_queue_.empty()) {
        std::string message_id = message_process_queue_.front();
        auto message = message_map_.at(message_id);

        web::http::uri_builder builder(U(""));
        builder.append_query(U("message"), message.payload);
        builder.append_query(U("type"), message.type);

        bool status = false;

        try {
          http_client_ptr_->request(
            web::http::methods::POST, builder.to_string()).then(
            [message, &status](http_response response)
            {
              if (response.status_code() < 400) {
                RS_LOG_INFO(
                  "mobile notification sent for message with id: %s",
                  message.message_id.c_str());
                status = true;
              } else {
                RS_LOG_ERROR(
                  "mobile notification error for message with id: %s",
                  message.message_id.c_str());
                status = false;
              }
            }).wait();
        } catch (const std::exception & e) {
          RS_LOG_ERROR("mobile notification error: %s", e.what());
          break;
        }

        if (status) {
          message_process_queue_.pop();
        } else {
          break;
        }
      }

      notify_trigger_.wait(lock);
    }
  }
};

NotificationManager::NotificationManager(
  std::string websocket_uri,
  std::string mobile_uri)
: websocket_uri_(websocket_uri), mobile_uri_(mobile_uri)
{
  websocket_pimpl_ = std::make_unique<WebsocketImplementation>(
    websocket_uri_,
    message_map_);
  mobile_pimpl_ = std::make_unique<MobileImplementation>(
    mobile_uri_,
    message_map_);
}

std::string NotificationManager::publish_state(
  const std::string payload,
  const std::string type)
{
  Message message;
  message.payload = payload;
  message.type = type;
  message.message_id = rmf_scheduler::utils::gen_uuid();
  message_map_[message.message_id] = message;
  websocket_pimpl_->publish_state(message.message_id);
  mobile_pimpl_->publish_state(message.message_id);
  return message.message_id;
}

std::vector<std::pair<
    NotificationType, ConnectionState>> NotificationManager::check_connection_state()
{
  std::pair<NotificationType, ConnectionState> websocket_connection_state;
  websocket_connection_state.first = NotificationType::WEBSOCKET;
  websocket_connection_state.second = websocket_pimpl_->check_connection_state() ?
    ConnectionState::CONNECTED : ConnectionState::DISCONNECTED;

  std::pair<NotificationType, ConnectionState> mobile_connection_state;
  mobile_connection_state.first = NotificationType::REST;
  mobile_connection_state.second = mobile_pimpl_->check_connection_state() ?
    ConnectionState::CONNECTED : ConnectionState::DISCONNECTED;

  std::vector<std::pair<NotificationType, ConnectionState>> results;
  results.push_back(websocket_connection_state);
  results.push_back(mobile_connection_state);

  return results;
}

void NotificationManager::clear_messages()
{
  message_map_.clear();
}

NotificationManager::~NotificationManager() {}

}  // namespace rmf_notification
