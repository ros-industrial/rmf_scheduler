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
#include "rmf_api_msgs/schemas/task_estimate_result.hpp"
#include "rmf_api_msgs/schemas/estimate_robot_task_request.hpp"
#include "rmf_api_msgs/schemas/estimate_robot_task_response.hpp"
#include "rmf_api_msgs/schemas/robot_task_request.hpp"
#include "rmf_api_msgs/schemas/robot_task_response.hpp"
#include "rmf_api_msgs/schemas/dispatch_task_request.hpp"
#include "rmf_api_msgs/schemas/dispatch_task_response.hpp"
#include "rmf_api_msgs/schemas/task_state.hpp"
#include "rmf_api_msgs/schemas/error.hpp"

#include "rmf_scheduler_plugins/robot_task_estimate_client.hpp"

namespace rmf_scheduler_plugins
{

static constexpr char RMF_TASK_API_REQUESTS_TOPIC[] = "/task_api_requests";
static constexpr char RMF_TASK_API_RESPONSES_TOPIC[] = "/task_api_responses";

RobotTaskEstimateClient::RobotTaskEstimateClient()
: schema_validator_(std::vector<nlohmann::json>{
    rmf_api_msgs::schemas::error,
    rmf_api_msgs::schemas::task_request,
    rmf_api_msgs::schemas::task_state,
    rmf_api_msgs::schemas::dispatch_task_request,
    rmf_api_msgs::schemas::dispatch_task_response,
    rmf_api_msgs::schemas::robot_task_request,
    rmf_api_msgs::schemas::robot_task_response,
    rmf_api_msgs::schemas::task_estimate_result,
    rmf_api_msgs::schemas::estimate_robot_task_request,
    rmf_api_msgs::schemas::estimate_robot_task_response,
  })
{
}


void RobotTaskEstimateClient::init(
  const std::shared_ptr<void> & node)
{
  node_ = std::static_pointer_cast<rclcpp::Node>(node);

  // QOS settings
  auto default_qos = rclcpp::SystemDefaultsQoS();

  auto transient_local_qos = rclcpp::SystemDefaultsQoS()
    .reliable()
    .transient_local()
    .keep_last(100);

  request_publisher_ =
    node_->create_publisher<rmf_task_msgs::msg::ApiRequest>(
    RMF_TASK_API_REQUESTS_TOPIC,
    transient_local_qos);

  response_subscriber_ =
    node_->create_subscription<rmf_task_msgs::msg::ApiResponse>(
    RMF_TASK_API_RESPONSES_TOPIC,
    default_qos,
    std::bind(
      &RobotTaskEstimateClient::handle_response, this,
      std::placeholders::_1));
}


std::shared_future<rmf_scheduler::task::EstimateResponse>
RobotTaskEstimateClient::async_estimate(
  const std::string & id,
  const rmf_scheduler::task::EstimateRequest & request)
{
  std::lock_guard<std::mutex> lk(mutex_);
  // Get validation error if any
  // TODO(Briancbn): exception handling
  nlohmann::json estimate_request;
  estimate_request["request"] = request.details["request"];

  if (request.state != nullptr) {
    nlohmann::json state;
    state["waypoint"] = request.state->waypoint;
    state["orientation"] = request.state->orientation;
    state["battery_soc"] = request.state->consumables["battery_soc"];
    state["time"] = static_cast<uint64_t>(request.start_time / 1e6);
    estimate_request["initial_state"] = state;
  }

  estimate_request["type"] = "estimate_task_request";

  // change them to slugs
  // all spaces(" ") and dash ("-") to underline ("_")
  estimate_request["robot"] = request.details["robot"].get<std::string>();
  estimate_request["fleet"] = request.details["fleet"].get<std::string>();
  std::string error;

  auto json_uri = nlohmann::json_uri{
    rmf_api_msgs::schemas::estimate_robot_task_request["$id"]
  };
  schema_validator_.validate(json_uri, estimate_request);

  response_map_[id] = std::promise<rmf_scheduler::task::EstimateResponse>();

  // Publish request
  request_publisher_->publish(
    rmf_task_msgs::build<rmf_task_msgs::msg::ApiRequest>()
    .json_msg(estimate_request.dump())
    .request_id(id)
  );

  return response_map_.at(id).get_future();
}

void RobotTaskEstimateClient::handle_response(
  const rmf_task_msgs::msg::ApiResponse & msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  auto response_map_itr = response_map_.find(msg.request_id);
  if (response_map_itr == response_map_.end()) {
    return;
  }

  nlohmann::json raw_response;
  try {
    raw_response = nlohmann::json::parse(msg.json_msg);
    auto json_uri = nlohmann::json_uri{
      rmf_api_msgs::schemas::estimate_robot_task_request["$id"]
    };
    schema_validator_.validate(json_uri, raw_response);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Error in response to [%s]: %s",
      msg.request_id.c_str(),
      e.what());
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Got Response to [%s]: %s",
    msg.request_id.c_str(),
    msg.json_msg.c_str());

  if (!raw_response["success"].get<bool>()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Error in response to to [%s]",
      msg.request_id.c_str());
    return;
  }

  rmf_scheduler::task::EstimateResponse response;
  nlohmann::json result = raw_response["result"];
  response.deployment_time =
    static_cast<uint64_t>(result["deployment_time"].get<int>()) * 1e6;
  response.finish_time =
    static_cast<uint64_t>(result["finish_time"].get<int>()) * 1e6;
  response.duration =
    static_cast<uint64_t>(result["duration"].get<int>()) * 1e6;

  response.state.waypoint = result["state"]["waypoint"].get<int>();
  response.state.orientation = result["state"]["orientation"].get<double>();
  response.state.consumables["battery_soc"] =
    result["state"]["battery_soc"].get<double>();

  response_map_itr->second.set_value(response);
  response_map_.erase(response_map_itr);
}

}  // namespace rmf_scheduler_plugins


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rmf_scheduler_plugins::RobotTaskEstimateClient, rmf_scheduler::task::EstimateInterface)
