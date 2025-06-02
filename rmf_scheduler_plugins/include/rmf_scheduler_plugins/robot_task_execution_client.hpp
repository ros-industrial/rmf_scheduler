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

#ifndef RMF_SCHEDULER_PLUGINS__ROBOT_TASK_EXECUTION_CLIENT_HPP_
#define RMF_SCHEDULER_PLUGINS__ROBOT_TASK_EXECUTION_CLIENT_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <chrono>
#include <tuple>

#include "rmf_scheduler/task/execution_interface.hpp"
#include "rmf_scheduler/schema_validator.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rmf_task_msgs/msg/api_request.hpp"
#include "rmf_task_msgs/msg/api_response.hpp"

using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

namespace rmf_scheduler_plugins
{

class RobotTaskExecutionClient : public rmf_scheduler::task::ExecutionInterface
{
public:
  RobotTaskExecutionClient();
  virtual ~RobotTaskExecutionClient();

  void init(const std::shared_ptr<void> & node) override;

  void start(
    const std::string & id,
    const nlohmann::json & task_details) override;

  void pause(const std::string & id) override;

  void resume(const std::string & id) override;

  void cancel(const std::string & id) override;

private:
  std::thread task_state_health_update_thread_;

  std::mutex task_status_map_mutex_;

  rmf_scheduler::SchemaValidator schema_validator_;

  rclcpp::Node::SharedPtr node_;

  void update_health();

  void update_loop();

  void handle_response(
    const std_msgs::msg::String & response);

  std::unordered_map<std::string, std::tuple<std::string, TimePoint, double>> task_status_map_;

  rclcpp::Publisher<rmf_task_msgs::msg::ApiRequest>::SharedPtr task_request_publisher_;

  rclcpp::Publisher<rmf_task_msgs::msg::ApiRequest>::SharedPtr pause_resume_request_publisher_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_states_sub_;
};

}  // namespace rmf_scheduler_plugins

#endif  // RMF_SCHEDULER_PLUGINS__ROBOT_TASK_EXECUTION_CLIENT_HPP_
