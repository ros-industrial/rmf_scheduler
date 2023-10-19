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

#include <thread>

#include "cpprest/http_client.h"

#include "rmf_notification/http_client.hpp"
#include "rmf_scheduler/log.hpp"

namespace rmf_notification
{

class HTTPClient::Impl
{
public:
  using http_request = web::http::http_request;
  using http_response = web::http::http_response;
  using http_client = web::http::client::http_client;

  void init(const std::string & url)
  {
    // creating http client instance
    http_client_ptr_ = std::make_shared<http_client>(U(url));
    // starting new thread for handling request process in background
    shutdown_ = false;
    connected_ = false;
    processing_thread_ = std::thread(
      [this]()
      {
        RS_LOG_INFO(
          "starting mobile notification background runner in processing thread");
        runner();
      });
  }

  void publish(const Message & message)
  {
    RS_LOG_INFO(
      "mobile notification request raised for message with id: %s",
      message.message_id.c_str());
    std::unique_lock<std::mutex> lock(notify_mtx_);
    message_process_queue_.push(message);
    notify_trigger_.notify_one();
  }

  bool is_connected() const
  {
    return connected_;
  }

  bool is_shutdown()
  {
    return shutdown_;
  }

  void shutdown()
  {
    shutdown_ = true;
    notify_trigger_.notify_one();
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    connected_ = false;
  }

private:
  void runner()
  {
    while (!shutdown_) {
      std::unique_lock<std::mutex> lock(notify_mtx_);

      while (!message_process_queue_.empty()) {
        auto message = message_process_queue_.front();

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
                  "HTTP notification sent for message with id: %s",
                  message.message_id.c_str());
                status = true;
              } else {
                RS_LOG_ERROR(
                  "HTTP notification error for message with id: %s",
                  message.message_id.c_str());
                status = false;
              }
            }).wait();
        } catch (const std::exception & e) {
          RS_LOG_ERROR("HTTP notification error: %s", e.what());
          connected_ = false;
          break;
        }

        connected_ = status;
        if (status) {
          message_process_queue_.pop();
        } else {
          break;
        }
      }

      notify_trigger_.wait(lock);
    }
  }
  std::shared_ptr<http_client> http_client_ptr_;
  std::queue<Message> message_process_queue_;
  std::mutex notify_mtx_;
  std::condition_variable notify_trigger_;
  std::thread processing_thread_;
  std::atomic_bool connected_ = false;
  std::atomic_bool shutdown_ = true;
};

HTTPClient::HTTPClient()
: impl_ptr_(std::make_unique<Impl>())
{
}

HTTPClient::~HTTPClient()
{
  if (!impl_ptr_->is_shutdown()) {
    impl_ptr_->shutdown();
  }
}

void HTTPClient::init(const std::string & endpoint_uri)
{
  return impl_ptr_->init(endpoint_uri);
}

bool HTTPClient::is_connected() const
{
  return impl_ptr_->is_connected();
}

void HTTPClient::publish(const Message & message)
{
  impl_ptr_->publish(message);
}

void HTTPClient::shutdown()
{
  impl_ptr_->shutdown();
}

}  // namespace rmf_notification
