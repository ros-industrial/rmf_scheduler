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
#include "rmf_api_msgs/schemas/simple_response.hpp"
#include "rmf_api_msgs/schemas/cancel_task_request.hpp"
#include "rmf_api_msgs/schemas/cancel_task_response.hpp"
#include "rmf_api_msgs/schemas/task_state.hpp"
#include "rmf_api_msgs/schemas/dispatch_task_response.hpp"
#include "rmf_api_msgs/schemas/robot_task_request.hpp"
#include "rmf_api_msgs/schemas/robot_task_response.hpp"
#include "rmf_api_msgs/schemas/token_response.hpp"
#include "rmf_api_msgs/schemas/interrupt_task_request.hpp"
#include "rmf_api_msgs/schemas/interrupt_task_response.hpp"
#include "rmf_api_msgs/schemas/resume_task_request.hpp"
#include "rmf_api_msgs/schemas/resume_task_response.hpp"
#include "rmf_api_msgs/schemas/error.hpp"

#include "rmf_scheduler_plugins/robot_task_execution_client.hpp"

namespace rmf_scheduler_plugins
{

static constexpr char RMF_TASK_API_REQUEST_TOPIC[] = "/task_api_requests";
static constexpr char RMF_TASK_API_RESPONSE_TOPIC[] = "/task_api_responses";

RobotTaskExecutionClient::RobotTaskExecutionClient()
: schema_validator_({
    rmf_api_msgs::schemas::error,
    rmf_api_msgs::schemas::task_request,
    rmf_api_msgs::schemas::simple_response,
    rmf_api_msgs::schemas::cancel_task_request,
    rmf_api_msgs::schemas::cancel_task_response,
    rmf_api_msgs::schemas::task_state,
    rmf_api_msgs::schemas::dispatch_task_response,
    rmf_api_msgs::schemas::robot_task_request,
    rmf_api_msgs::schemas::robot_task_response,
    rmf_api_msgs::schemas::token_response,
    rmf_api_msgs::schemas::interrupt_task_request,
    rmf_api_msgs::schemas::interrupt_task_response,
    rmf_api_msgs::schemas::resume_task_request,
    rmf_api_msgs::schemas::resume_task_response,
  })
{
}

RobotTaskExecutionClient::~RobotTaskExecutionClient()
{
}

void RobotTaskExecutionClient::init(
  const std::shared_ptr<void> & node)
{
  node_ = std::static_pointer_cast<rclcpp::Node>(node);

  // QOS settings
  auto default_qos = rclcpp::SystemDefaultsQoS();

  auto transient_local_qos = rclcpp::SystemDefaultsQoS()
    .reliable()
    .transient_local()
    .keep_last(100);

  task_request_publisher_ =
    node_->create_publisher<rmf_task_msgs::msg::ApiRequest>(
    RMF_TASK_API_REQUEST_TOPIC,
    transient_local_qos);

  task_states_sub_ =
    node_->create_subscription<rmf_task_msgs::msg::ApiResponse>(
    "/task_states",
    default_qos,
    std::bind(
      &RobotTaskExecutionClient::handle_response, this,
      std::placeholders::_1));
}

void RobotTaskExecutionClient::start(
  const std::string & id,
  const nlohmann::json & task_details)
{
  try {
    RCLCPP_INFO_STREAM(node_->get_logger(), task_details);
    nlohmann::json task_request;
    task_request["request"] = task_details["request"];
    task_request["robot"] = task_details["robot"].get<std::string>();
    task_request["fleet"] = task_details["fleet"].get<std::string>();
    task_request["type"] = "robot_task_request";
    auto json_uri = nlohmann::json_uri{
      rmf_api_msgs::schemas::robot_task_request["$id"]
    };
    schema_validator_.validate(json_uri, task_request);

    task_status_map_[id] = "idle";

    task_request_publisher_->publish(
      rmf_task_msgs::build<rmf_task_msgs::msg::ApiRequest>()
      .json_msg(task_request.dump())
      .request_id(id)
    );
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error in start request with id: [%s]: %s",
      id.c_str(),
      e.what());
  }
}

void RobotTaskExecutionClient::pause(const std::string & id)
{
  try {
    std::string modified_id = "pause_" + id;
    nlohmann::json interrupt_task_request_json = {{"task_id", id}};
    auto json_uri = nlohmann::json_uri{
      rmf_api_msgs::schemas::interrupt_task_request["$id"]
    };
    schema_validator_.validate(json_uri, interrupt_task_request_json);

    task_request_publisher_->publish(
      rmf_task_msgs::build<rmf_task_msgs::msg::ApiRequest>()
      .json_msg(interrupt_task_request_json.dump())
      .request_id(modified_id)
    );
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error in pause request with id: [%s]: %s",
      id.c_str(),
      e.what());
  }
}

void RobotTaskExecutionClient::resume(const std::string & id)
{
  try {
    std::string modified_id = "resume_" + id;
    nlohmann::json resume_task_request_json = {{"task_id", id}};
    auto json_uri = nlohmann::json_uri{
      rmf_api_msgs::schemas::resume_task_request["$id"]
    };
    schema_validator_.validate(json_uri, resume_task_request_json);

    task_request_publisher_->publish(
      rmf_task_msgs::build<rmf_task_msgs::msg::ApiRequest>()
      .json_msg(resume_task_request_json.dump())
      .request_id(modified_id)
    );
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error in resume request with id: [%s]: %s",
      id.c_str(),
      e.what());
  }
}

void RobotTaskExecutionClient::cancel(const std::string & id)
{
  try {
    std::string modified_id = "cancel_" + id;
    nlohmann::json cancel_task_request_json = {{"task_id", id}};
    auto json_uri = nlohmann::json_uri{
      rmf_api_msgs::schemas::cancel_task_request["$id"]
    };
    schema_validator_.validate(json_uri, cancel_task_request_json);

    task_request_publisher_->publish(
      rmf_task_msgs::build<rmf_task_msgs::msg::ApiRequest>()
      .json_msg(cancel_task_request_json.dump())
      .request_id(modified_id)
    );
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error in cancel request with id: [%s]: %s",
      id.c_str(),
      e.what());
  }
}

void RobotTaskExecutionClient::handle_response(
  const rmf_task_msgs::msg::ApiResponse & response)
{
  auto task_status_itr = task_status_map_.find(response.request_id);
  if (task_status_itr == task_status_map_.end()) {
    return;
  }

  nlohmann::json task_state_json;
  try {
    task_state_json = nlohmann::json::parse(response.json_msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error when parsing task states for task [%s]%s",
      response.request_id.c_str(),
      e.what());
    return;
  }
  std::string status = task_state_json["status"].get<std::string>();
  if (status != task_status_itr->second) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Received new task status [%s] for task [%s];",
      status.c_str(),
      response.request_id.c_str());
    task_status_itr->second = status;
  }

  if (status == "completed") {
    notify_completion(response.request_id, true, "completed");
  }
}

}  // namespace rmf_scheduler_plugins


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rmf_scheduler_plugins::RobotTaskExecutionClient, rmf_scheduler::task::ExecutionInterface)
