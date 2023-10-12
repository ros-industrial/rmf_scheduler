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

#include "rclcpp/rclcpp.hpp"

#include "rmf_notification/message.hpp"
#include "rmf_notification/notification_manager.hpp"

const rclcpp::Logger LOGGER = rclcpp::get_logger("test_notification");

// this class is only for validation purposes. will be removed in future

class TestNotification : public rclcpp::Node
{
public:
  TestNotification()
  : Node("test_notification_node")
  {
    timer_ = create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&TestNotification::timer_callback, this));
    std::string websocket_uri = "ws://localhost:8000/_internal";
    std::string mobile_uri = "http://localhost:8000/notification/telegram";
    notification_manager_ptr_ = new rmf_notification::NotificationManager(
      websocket_uri,
      mobile_uri
    );
    RCLCPP_INFO(LOGGER, "Starting test notification node");
  }

  ~TestNotification()
  {
    timer_->cancel();
    delete notification_manager_ptr_;
  }

private:
  void timer_callback()
  {
    counter_ = counter_ + 1;
    std::string message = "test message: " + std::to_string(counter_);
    notification_manager_ptr_->publish_state(message, "maintenance_log_update");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rmf_notification::NotificationManager * notification_manager_ptr_;
  int counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNotification>());
  rclcpp::shutdown();
  return 0;
}
