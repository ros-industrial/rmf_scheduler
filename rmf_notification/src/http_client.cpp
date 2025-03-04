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
#include "rclcpp/logging.hpp"

namespace rmf_notification
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rmf_notification_http");

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
        RCLCPP_INFO(
          LOGGER,
          "starting mobile notification background runner in processing thread");
        runner();
      });
  }

  void init_keycloak_auth(
    const std::string & username,
    const std::string & password,
    const std::string & client_id,
    const std::string & url)
  {
    keycloak_auth_client_ptr_ = std::make_shared<http_client>(U(url));

    web::http::uri_builder builder(U(""));
    builder.append_query(U("username"), username);
    builder.append_query(U("password"), password);
    builder.append_query(U("client_id"), client_id);
    builder.append_query(U("grant_type"), "password");
    http_request request(web::http::methods::POST);
    keycloak_query_ = builder.query();
    utility::string_t token;
    request_keycloak_token(token);
  }

  bool request_keycloak_token(utility::string_t & token)
  {
    RCLCPP_INFO(LOGGER, "Retrieving access token from keycloak");
    web::json::value json_return;
    keycloak_auth_client_ptr_->request(
      web::http::methods::POST, U(""),
      keycloak_query_,
      U("application/x-www-form-urlencoded"))
    .then(
      [](const web::http::http_response & response) {
        return response.extract_json();
      })
    .then(
      [&json_return](const pplx::task<web::json::value> & task) {
        try {
          json_return = task.get();
        } catch (const web::http::http_exception & e) {
          std::cout << "error " << e.what() << std::endl;
        }
      })
    .wait();

    if (json_return.has_field("access_token")) {
      token = json_return["access_token"].as_string();
      return true;
    } else {
      return false;
    }
  }

  void publish(const Message & message)
  {
    RCLCPP_INFO(
      LOGGER,
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

        http_request msg(web::http::methods::POST);
        web::http::uri_builder builder(U(""));
        builder.append_query(U("message"), message.payload);
        builder.append_query(U("type"), message.type);
        msg.set_request_uri(builder.to_string());
        bool status = false;

        try {
          // Retrieve keycloak token if enabled
          if (keycloak_auth_client_ptr_) {
            utility::string_t token;
            if (request_keycloak_token(token)) {
              msg.headers().add(U("Authorization"), U("Bearer ") + token);
            }
          }
          http_client_ptr_->request(msg).then(
            [message, &status](http_response response)
            {
              if (response.status_code() < 400) {
                RCLCPP_INFO(
                  LOGGER,
                  "HTTP notification sent for message with id: %s",
                  message.message_id.c_str());
                status = true;
              } else {
                RCLCPP_INFO(
                  LOGGER,
                  "HTTP notification error for message with id: %s",
                  message.message_id.c_str());
                status = false;
              }
            }).wait();
        } catch (const std::exception & e) {
          RCLCPP_INFO(LOGGER, "HTTP notification error: %s", e.what());
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
  std::shared_ptr<http_client> keycloak_auth_client_ptr_;
  utility::string_t keycloak_query_;
  std::queue<Message> message_process_queue_;
  std::mutex notify_mtx_;
  std::condition_variable notify_trigger_;
  std::thread processing_thread_;
  std::atomic_bool connected_ = false;
  std::atomic_bool shutdown_ = true;
};

HTTPClient::HTTPClient(
  const std::string & auth,
  const std::string & username,
  const std::string & password,
  const std::string & client_id,
  const std::string & token_url)
: impl_ptr_(std::make_unique<Impl>())
{
  if (auth == "keycloak") {
    impl_ptr_->init_keycloak_auth(username, password, client_id, token_url);
  }
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
