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

#include "rmf_api_msgs/schemas/task_request.hpp"
#include "rmf_api_msgs/schemas/task_estimate_request.hpp"
#include "rmf_api_msgs/schemas/task_estimate_response.hpp"
#include "rmf_api_msgs/schemas/robot_task_request.hpp"
#include "rmf_api_msgs/schemas/robot_task_response.hpp"
#include "rmf_api_msgs/schemas/dispatch_task_request.hpp"
#include "rmf_api_msgs/schemas/dispatch_task_response.hpp"
#include "rmf_api_msgs/schemas/task_state.hpp"
#include "rmf_api_msgs/schemas/error.hpp"

#include "rmf_scheduler_plugins/robot_task_estimate_client.hpp"

namespace rmf_scheduler_plugins
{

static constexpr char RMF_TASK_API_REQUEST_TOPIC[] = "/task_api_request";
static constexpr char RMF_TASK_API_RESPONSE_TOPIC[] = "/task_api_request";

RobotTaskEstimateClient::RobotTaskEstimateClient()
: support_task_types_({
    "rmf/robot_task"
  }),
  schema_validator_({
    rmf_api_msgs::schemas::task_request,
    rmf_api_msgs::schemas::dispatch_task_request,
    rmf_api_msgs::schemas::dispatch_task_response,
    rmf_api_msgs::schemas::robot_task_request,
    rmf_api_msgs::schemas::robot_task_response,
    rmf_api_msgs::schemas::task_estimate_request,
    rmf_api_msgs::schemas::task_estimate_response,
    rmf_api_msgs::schemas::task_state,
    rmf_api_msgs::schemas::error,
  })
{
}


void RobotTaskEstimateClient::init(
  const std::string & name,
  const std::string & ns)
{
  rmf_scheduler::EstimateInterface::init(name, ns);

  // QOS settings
  auto default_qos = rclcpp::SystemDefaultsQoS();

  auto transient_local_qos = rclcpp::SystemDefaultsQoS()
    .reliable()
    .transient_local()
    .keep_last(100);

  request_publisher_ =
    get_node()->create_publisher<rmf_task_msgs::msg::ApiRequest>(
    RMF_TASK_API_REQUEST_TOPIC,
    transient_local_qos);

  response_subscriber_ =
    get_node()->create_subscription<rmf_task_msgs::msg::ApiResponse>(
    RMF_TASK_API_RESPONSE_TOPIC,
    default_qos,
    std::bind(
      &RobotTaskEstimateClient::handle_response, this,
      std::placeholders::_1));
}


std::shared_future<nlohmann::json> RobotTaskEstimateClient::async_estimate(
  const std::string & id,
  const nlohmann::json & event_details)
{
  // Get validation error if any
  // TODO(Briancbn): exception handling
  std::string error;
  auto json_uri = nlohmann::json_uri{
    rmf_api_msgs::schemas::task_estimate_request["$id"]
  };
  schema_validator_.validate(json_uri, event_details);

  response_map_[id] = std::promise<nlohmann::json>();

  // Publish request
  request_publisher_->publish(
    rmf_task_msgs::build<rmf_task_msgs::msg::ApiRequest>()
    .json_msg(event_details.dump())
    .request_id(id)
  );

  return response_map_.at(id).get_future();
}


const std::unordered_set<std::string> & RobotTaskEstimateClient::support_task_types() const
{
  return support_task_types_;
}


void RobotTaskEstimateClient::handle_response(
  const rmf_task_msgs::msg::ApiResponse & msg)
{
  auto response_map_itr = response_map_.find(msg.request_id);
  if (response_map_itr == response_map_.end()) {
    return;
  }

  nlohmann::json response;
  try {
    response = nlohmann::json::parse(msg.json_msg);
    auto json_uri = nlohmann::json_uri{
      rmf_api_msgs::schemas::task_estimate_response["$id"]
    };
    schema_validator_.validate(json_uri, response);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Error in response to [%s]: %s",
      msg.request_id.c_str(),
      e.what());
    return;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Got Response to [%s]: %s",
    msg.request_id.c_str(),
    msg.json_msg.c_str());

  response_map_itr->second.set_value(response);
  response_map_.erase(response_map_itr);
}

}  // namespace rmf_scheduler_plugins


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rmf_scheduler_plugins::RobotTaskEstimateClient, rmf_scheduler::EstimateInterface)
