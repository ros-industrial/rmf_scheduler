#include <memory>
#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>
#include <optional>
#include <future>

#include <rclcpp/rclcpp.hpp>

#include <rmf_task_msgs/msg/api_request.hpp>
#include <rmf_task_msgs/msg/api_response.hpp>
#include <rmf_api_msgs/schemas/task_request.hpp>
#include <rmf_api_msgs/schemas/task_estimate_request.hpp>
#include <rmf_api_msgs/schemas/task_estimate_response.hpp>
#include <rmf_api_msgs/schemas/robot_task_request.hpp>
#include <rmf_api_msgs/schemas/robot_task_response.hpp>
#include <rmf_api_msgs/schemas/dispatch_task_request.hpp>
#include <rmf_api_msgs/schemas/dispatch_task_response.hpp>
#include <rmf_api_msgs/schemas/task_state.hpp>
#include <rmf_api_msgs/schemas/error.hpp>

#ifndef TASK_ESTIMATE_CLIENT_HPP
#define TASK_ESTIMATE_CLIENT_HPP

class TaskEstimateClient : public std::enable_shared_from_this<TaskEstimateClient>
{
private:
  int estimate_count = 0;

  int task_count = 0;

  rclcpp::Node::SharedPtr _node;

  std::unordered_map<std::string, nlohmann::json> schema_dictionary = {};

  std::unordered_map<std::string,
    std::pair<std::string, std::promise<nlohmann::json>>> response_map = {};

  rclcpp::Publisher<rmf_task_msgs::msg::ApiRequest>::SharedPtr request_publisher;

  rclcpp::Subscription<rmf_task_msgs::msg::ApiResponse>::SharedPtr
    response_subscriber;

  TaskEstimateClient(rclcpp::Node::SharedPtr node);

  nlohmann::json_schema::json_validator make_validator(
    const nlohmann::json & schema) const;

  void schema_loader(
    const nlohmann::json_uri & id, nlohmann::json & value) const;

  bool validate_json(
    const nlohmann::json & json,
    const nlohmann::json_schema::json_validator & validator,
    std::string & error) const;

  bool validate_and_publish_request(
    const nlohmann::json & request_json,
    const nlohmann::json_schema::json_validator & validator,
    const std::string & request_id,
    const std::string & schema_response_id);

  void validate_response(
    const nlohmann::json & response,
    const nlohmann::json_schema::json_validator & validator,
    std::string request_id);

  void handle_response(
    const rmf_task_msgs::msg::ApiResponse & request);

public:
  static std::shared_ptr<TaskEstimateClient> make(rclcpp::Node::SharedPtr node);

  std::shared_future<nlohmann::json> async_send_request(
    const nlohmann::json & request_json);

  std::shared_future<nlohmann::json> async_dispatch_request(
    const nlohmann::json & request_json);

  rclcpp::Node::SharedPtr node();
};

using TaskEstimateClientPtr = std::shared_ptr<TaskEstimateClient>;

#endif
