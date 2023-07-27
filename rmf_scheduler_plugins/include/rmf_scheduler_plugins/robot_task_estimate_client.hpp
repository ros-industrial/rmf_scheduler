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

#ifndef RMF_SCHEDULER_PLUGINS__ROBOT_TASK_ESTIMATE_CLIENT_HPP_
#define RMF_SCHEDULER_PLUGINS__ROBOT_TASK_ESTIMATE_CLIENT_HPP_

#include <memory>
#include <future>
#include <string>
#include <unordered_set>
#include <unordered_map>

#include "nlohmann/json.hpp"
#include "nlohmann/json-schema.hpp"
#include "rmf_scheduler/estimate_interface.hpp"
#include "rmf_scheduler/schema_validator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmf_task_msgs/msg/api_request.hpp"
#include "rmf_task_msgs/msg/api_response.hpp"

namespace rmf_scheduler_plugins
{

class RobotTaskEstimateClient : public rmf_scheduler::EstimateInterface
{
public:
  RobotTaskEstimateClient();
  virtual ~RobotTaskEstimateClient() {}

  void init(
    const std::string & name,
    const std::string & ns) override;

  std::shared_future<nlohmann::json> async_estimate(
    const std::string & id,
    const nlohmann::json & event_details) override;

  const std::unordered_set<std::string> & support_task_types() const override;

private:
  void handle_response(
    const rmf_task_msgs::msg::ApiResponse & request);

  const std::unordered_set<std::string> support_task_types_;

  rmf_scheduler::SchemaValidator schema_validator_;

  std::unordered_map<std::string, std::promise<nlohmann::json>> response_map_;

  rclcpp::Publisher<rmf_task_msgs::msg::ApiRequest>::SharedPtr request_publisher_;

  rclcpp::Subscription<rmf_task_msgs::msg::ApiResponse>::SharedPtr
    response_subscriber_;
};

}  // namespace rmf_scheduler_plugins

#endif  // RMF_SCHEDULER_PLUGINS__ROBOT_TASK_ESTIMATE_CLIENT_HPP_
