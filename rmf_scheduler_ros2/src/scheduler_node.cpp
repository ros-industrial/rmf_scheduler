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
#include "rmf_scheduler_ros2/utils.hpp"

namespace rmf_scheduler_ros2
{

SchedulerNode::SchedulerNode()
{
}

SchedulerNode::SharedPtr SchedulerNode::make_node(
  rclcpp::Node::SharedPtr node,
  const rmf_scheduler::SchedulerOptions::DynamicChargerMap & dynamic_charger_map,
  const rmf_scheduler::SchedulerOptions::FixedChargerMap & fixed_charger_map)
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

  // Default parameters
  double tick_period = 5 * 60;  // 5min
  double allow_past_events_duration = 5 * 60;  // 5min
  double series_max_expandable_duration = 2 * 30 * 24 * 60 * 60;  // 2months
  bool expand_series = true;
  double estimate_timeout = 2.0;  // 2s

  bool enable_optimization = false;
  std::string optimization_window = "";
  std::string optimization_window_timezone = "Asia/Singapore";

  bool enable_local_caching = false;
  std::string cache_dir = ".";
  int cache_keep_last = 5;

  // Get parameters for scheduler options
  declare_or_get_param<double>(
    tick_period, "tick_period",
    node, node->get_logger(),
    tick_period);

  declare_or_get_param<double>(
    allow_past_events_duration, "allow_past_events_duration",
    node, node->get_logger(),
    allow_past_events_duration);

  declare_or_get_param<double>(
    series_max_expandable_duration, "series_max_expandable_duration",
    node, node->get_logger(),
    series_max_expandable_duration);

  declare_or_get_param<bool>(
    expand_series, "expand_series",
    node, node->get_logger(),
    expand_series);

  declare_or_get_param<double>(
    estimate_timeout, "estimate_timeout",
    node, node->get_logger(),
    estimate_timeout);

  declare_or_get_param<bool>(
    enable_optimization, "enable_optimization",
    node, node->get_logger(),
    enable_optimization);

  declare_or_get_param<std::string>(
    optimization_window, "optimization_window",
    node, node->get_logger(),
    optimization_window);

  declare_or_get_param<std::string>(
    optimization_window_timezone, "optimization_window_timezone",
    node, node->get_logger(),
    optimization_window_timezone);

  declare_or_get_param<bool>(
    enable_local_caching, "enable_local_caching",
    node, node->get_logger(),
    enable_local_caching);

  declare_or_get_param<std::string>(
    cache_dir, "cache_dir",
    node, node->get_logger(),
    cache_dir);

  declare_or_get_param<int>(
    cache_keep_last, "cache_keep_last",
    node, node->get_logger(),
    cache_keep_last);

  // Create Scheduler
  scheduler_node->_scheduler = std::make_shared<rmf_scheduler::Scheduler>(
    rmf_scheduler::SchedulerOptions()
    .tick_period(tick_period)
    .allow_past_events_duration(allow_past_events_duration)
    .series_max_expandable_duration(series_max_expandable_duration)
    .expand_series_automatically(expand_series)
    .estimate_timeout(estimate_timeout)
    .enable_optimization(enable_optimization)
    .optimization_window(optimization_window)
    .optimization_window_timezone(optimization_window_timezone)
    .enable_local_caching(enable_local_caching)
    .cache_dir(cache_dir)
    .cache_keep_last(static_cast<size_t>(cache_keep_last))
    .dynamic_charger_map(dynamic_charger_map)
    .fixed_charger_map(fixed_charger_map)
  );
  RCLCPP_INFO(scheduler_node->node()->get_logger(), "Scheduler node created.");

  return scheduler_node;
}

SchedulerNode::SharedPtr SchedulerNode::make_node(
  rclcpp::Node::SharedPtr scheduler_node)
{
  return make_node(scheduler_node, {}, {});
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
  } else if (type == "update_series") {
    RCLCPP_INFO(_node->get_logger(), "Received Update Series Request.");
    auto error_code = _scheduler->handle_update_series(*payload_it);
    RCLCPP_INFO(_node->get_logger(), "%s", error_code.str().c_str());
    response_msg.json_msg = error_code.json();
  } else if (type == "get") {
    RCLCPP_DEBUG(_node->get_logger(), "Received Get Schedule Request.");
    response_msg.json_msg = _scheduler->handle_get_schedule(*payload_it).dump();
  } else if (type == "delete") {
    RCLCPP_INFO(_node->get_logger(), "Received Delete Schedule Request.");
    auto error_code = _scheduler->handle_delete_schedule(*payload_it);
    response_msg.json_msg = error_code.json();
  } else {
    // skip
    // RCLCPP_INFO(_node->get_logger(), "Received Invalid Schedule Request [%s].", type.c_str());
    return;
  }

  _scheduler_response_publisher->publish(response_msg);
  RCLCPP_DEBUG(_node->get_logger(), "Response: %s", response_msg.json_msg.c_str());
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
