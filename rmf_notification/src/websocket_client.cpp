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

#include "websocketpp/config/asio_client.hpp"
#include "websocketpp/client.hpp"

#include "nlohmann/json.hpp"

#include "rmf_notification/websocket_client.hpp"
#include "rmf_scheduler/log.hpp"

namespace rmf_notification
{

class WebsocketClient::Impl
{
public:
  using asio_client = websocketpp::config::asio_client;

  void init(const std::string & websocket_uri)
  {
    websocket_uri_ = websocket_uri;
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
      websocket_client_ptr_->get_connection(websocket_uri, ec_for_new));
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
    shutdown_ = false;
    processing_thread_ = std::thread(
      [this]()
      {
        RS_LOG_INFO("starting websocket background runner in processing thread");
        runner();
      });
  }

  void publish(const Message & message)
  {
    RS_LOG_INFO(
      "websocket notification request raised for message with id: %s",
      message.message_id.c_str());
    std::unique_lock<std::mutex> lock(notify_mtx_);
    message_process_queue_.push(message);
    notify_trigger_.notify_one();
  }

  bool is_connected() const
  {
    return connected_;
  }

  bool is_shutdown() const
  {
    return shutdown_;
  }

  void shutdown()
  {
    shutdown_ = true;
    // triggering continuation of background runner for starting shutdown process
    notify_trigger_.notify_one();
    // closing websocket connection
    if (connected_) {
      websocket_client_ptr_->close(
        hdl_ptr_,
        websocketpp::close::status::going_away,
        "");
    }
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
        auto message = message_process_queue_.front();

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

  std::string websocket_uri_;
  std::unique_ptr<websocketpp::client<asio_client>> websocket_client_ptr_;
  std::weak_ptr<void> hdl_ptr_;
  std::queue<Message> message_process_queue_;
  std::mutex notify_mtx_;
  std::condition_variable notify_trigger_;
  std::thread processing_thread_;
  std::thread client_thread_;

  std::atomic_bool connected_ = false;
  std::atomic_bool shutdown_ = true;
};

WebsocketClient::WebsocketClient()
: impl_ptr_(std::make_unique<Impl>())
{
}

WebsocketClient::~WebsocketClient()
{
  if (!impl_ptr_->is_shutdown()) {
    impl_ptr_->shutdown();
  }
}

void WebsocketClient::init(
  const std::string & endpoint_uri)
{
  impl_ptr_->init(endpoint_uri);
}

bool WebsocketClient::is_connected() const
{
  return impl_ptr_->is_connected();
}

void WebsocketClient::publish(const Message & message)
{
  impl_ptr_->publish(message);
}

void WebsocketClient::shutdown()
{
  impl_ptr_->shutdown();
}

}  // namespace rmf_notification
