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


#include "rmf_scheduler_ros2/scheduler_node.hpp"
#include "rmf_scheduler/utils/system_time_utils.hpp"

namespace rmf_scheduler_ros2
{

SchedulerNode::SchedulerNode()
{
}

SchedulerNode::SharedPtr SchedulerNode::make_node(rclcpp::Node::SharedPtr node)
{
  // QOS settings
  auto default_qos = rclcpp::SystemDefaultsQoS();

  auto transient_local_qos = rclcpp::SystemDefaultsQoS()
    .reliable()
    .transient_local()
    .keep_last(100);

  SchedulerNode::SharedPtr scheduler_node(new SchedulerNode());

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

  scheduler_node->_test_deconflict_sub =
    scheduler_node->_node->create_subscription<std_msgs::msg::String>(
    "deconflict",
    default_qos,
    std::bind(
      &SchedulerNode::test_deconflict_cb, scheduler_node,
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

  // Start processing the request based on type
  // Create response and publish
  rmf_task_msgs::msg::ApiResponse response_msg;
  response_msg.request_id = msg.request_id;
  RCLCPP_DEBUG(_node->get_logger(), "Request: %s", payload_it->dump(2).c_str());

  std::string type = type_it->get<std::string>();
  if (type == "add") {
    RCLCPP_INFO(_node->get_logger(), "Received Add Schedule Request.");
    auto error_code = _scheduler->handle_add_schedule(*payload_it);
    response_msg.json_msg = error_code.json();
  } else if (type == "update") {
    RCLCPP_INFO(_node->get_logger(), "Received Update Schedule Request.");
    auto error_code = _scheduler->handle_update_schedule(*payload_it);
    response_msg.json_msg = error_code.json();
  } else if (type == "update_event_time") {
    RCLCPP_INFO(_node->get_logger(), "Received Update Event Time Request.");
    auto error_code = _scheduler->handle_update_event_time(*payload_it);
    response_msg.json_msg = error_code.json();
  } else if (type == "get") {
    RCLCPP_DEBUG(_node->get_logger(), "Received Get Schedule Request.");
    response_msg.json_msg = _scheduler->handle_get_schedule(*payload_it).dump();
  } else if (type == "delete") {
    RCLCPP_INFO(_node->get_logger(), "Received Delete Schedule Request.");
    auto error_code = _scheduler->handle_delete_schedule(*payload_it);
    response_msg.json_msg = error_code.json();
  } else {
    RCLCPP_INFO(_node->get_logger(), "Received Invalid Schedule Request [%s].", type.c_str());
    return;
  }

  _scheduler_response_publisher->publish(response_msg);
  RCLCPP_DEBUG(_node->get_logger(), "Response: %s", payload_it->dump().c_str());
}

void SchedulerNode::test_deconflict_cb(
  const std_msgs::msg::String & msg)
{
  rmf_scheduler::utils::set_timezone("Asia/Singapore");
  uint64_t start_time = rmf_scheduler::utils::from_localtime(msg.data + " 00:00:00 2023");
  uint64_t end_time = rmf_scheduler::utils::from_localtime(msg.data + " 23:59:59 2023");
  _scheduler->optimize(start_time, end_time);
}

rclcpp::Node::SharedPtr SchedulerNode::node() const
{
  return _node;
}

std::shared_ptr<rmf_scheduler::Scheduler> SchedulerNode::scheduler()
{
  return _scheduler;
}

std::shared_ptr<const rmf_scheduler::Scheduler> SchedulerNode::scheduler_const() const
{
  return _scheduler;
}

}  // namespace rmf_scheduler_ros2
