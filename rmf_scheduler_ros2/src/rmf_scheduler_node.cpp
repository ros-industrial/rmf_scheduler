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


#include "rmf_scheduler_ros2/rmf_scheduler_node.hpp"


SchedulerNode::SchedulerNode()
{
}

SchedulerNodePtr SchedulerNode::make_node(rclcpp::Node::SharedPtr node)
{
  // QOS settings
  auto default_qos = rclcpp::SystemDefaultsQoS();

  auto transient_local_qos = rclcpp::SystemDefaultsQoS()
    .reliable()
    .transient_local()
    .keep_last(100);

  SchedulerNodePtr scheduler_node(new SchedulerNode());

  scheduler_node->_node = node;

  scheduler_node->_scheduler_response_publisher =
    scheduler_node->_node->create_publisher<rmf_task_msgs::msg::ApiResponse>(
    "rmf_scheduler_api_responses",
    transient_local_qos);

  scheduler_node->_scheduler_request_subscriber =
    scheduler_node->_node->create_subscription<rmf_task_msgs::msg::ApiRequest>(
    "rmf_scheduler_api_requests",
    default_qos,
    std::bind(
      &SchedulerNode::schedule_request_cb, scheduler_node,
      std::placeholders::_1));

  // scheduler
  scheduler_node->_scheduler = std::make_shared<rmf_scheduler::Scheduler>();
  RCLCPP_INFO(scheduler_node->node()->get_logger(), "Scheduler node created.");

  return scheduler_node;
}

void SchedulerNode::schedule_request_cb(const rmf_task_msgs::msg::ApiRequest & msg)
{
  nlohmann::json request_json;
  try {
    request_json = nlohmann::json::parse(msg.json_msg);
  } catch (const std::exception & e) {
    rmf_task_msgs::msg::ApiResponse response_msg;
    rmf_scheduler::ErrorCode error_code = {
      rmf_scheduler::ErrorCode::FAILURE |
      rmf_scheduler::ErrorCode::INVALID_SCHEMA,
      e.what()
    };
    response_msg.request_id = msg.request_id;
    response_msg.json_msg = error_code.json();
    _scheduler_response_publisher->publish(response_msg);
  }

  // check if json has a type
  auto type_it = request_json.find("type");
  if (type_it == request_json.end()) {
    return;
  }

  // check if json has payload
  auto payload_it = request_json.find("payload");
  if (payload_it == request_json.end()) {
    return;
  }

  std::string type = type_it->get<std::string>();
  if (type == "add") {
    handle_add_request(msg.request_id, request_json["payload"]);
  } else if (type == "get") {
    handle_get_request(msg.request_id, request_json["payload"]);
  } else if (type == "update") {
    return;
  } else {
    return;
  }
}

void SchedulerNode::handle_add_request(const std::string & id, nlohmann::json & request)
{
  RCLCPP_INFO(_node->get_logger(), "Received Add Schedule Request.");
  RCLCPP_DEBUG(_node->get_logger(), "Request: %s", request.dump().c_str());
  rmf_scheduler::ErrorCode error_code = _scheduler->add_schedule(request);

  // Create response and publish
  rmf_task_msgs::msg::ApiResponse response_msg;
  response_msg.request_id = id;
  response_msg.json_msg = error_code.json();
  _scheduler_response_publisher->publish(response_msg);
}

void SchedulerNode::handle_get_request(const std::string & id, nlohmann::json & request)
{
  RCLCPP_INFO(_node->get_logger(), "Received Get Schedule Request.");

  // This string is alredy validated
  std::string schedule_string = _scheduler->get_schedule(request, 2);

  // Create response and publish
  rmf_task_msgs::msg::ApiResponse response_msg;
  response_msg.request_id = id;
  response_msg.json_msg = schedule_string;
  _scheduler_response_publisher->publish(response_msg);
}

rclcpp::Node::SharedPtr SchedulerNode::node()
{
  return _node;
}
