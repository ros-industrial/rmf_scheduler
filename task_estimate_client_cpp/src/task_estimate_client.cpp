#include "task_estimate_client_cpp/task_estimate_client.hpp"
#include <string>

TaskEstimateClient::TaskEstimateClient(rclcpp::Node::SharedPtr node)
{
  _node = node;

  // QOS settings
  auto default_qos = rclcpp::SystemDefaultsQoS();

  auto transient_local_qos = rclcpp::SystemDefaultsQoS()
    .reliable()
    .transient_local()
    .keep_last(100);

  request_publisher =
    _node->create_publisher<rmf_task_msgs::msg::ApiRequest>(
    "task_api_requests",
    transient_local_qos);

  response_subscriber =
    _node->create_subscription<rmf_task_msgs::msg::ApiResponse>(
    "task_api_responses",
    default_qos,
    std::bind(
      &TaskEstimateClient::handle_response, this,
      std::placeholders::_1));
}

TaskEstimateClientPtr TaskEstimateClient::make(rclcpp::Node::SharedPtr node)
{
  // Create publishers and subscribers
  auto task_estimate_cl_ptr = TaskEstimateClientPtr(
    new TaskEstimateClient(node));

  const std::vector<nlohmann::json> schemas = {
    rmf_api_msgs::schemas::task_request,
    rmf_api_msgs::schemas::task_estimate_request,
    rmf_api_msgs::schemas::task_estimate_response
  };

  for (const auto & schema: schemas) {
    const auto json_uri = nlohmann::json_uri{schema["$id"]};
    task_estimate_cl_ptr->schema_dictionary.insert({json_uri.url(), schema});
  }
  return task_estimate_cl_ptr;
}

// Takes in schema and returns returns validator
nlohmann::json_schema::json_validator TaskEstimateClient::make_validator(
  const nlohmann::json & schema) const
{
  return nlohmann::json_schema::json_validator(
    schema,
    [w = weak_from_this()](const nlohmann::json_uri & id,
    nlohmann::json & value)
    {
      const auto self = w.lock();
      if (!self) {
        return;
      }
      self->schema_loader(id, value);
    });
}

void TaskEstimateClient::schema_loader(
  const nlohmann::json_uri & id, nlohmann::json & value) const
{
  const auto it = schema_dictionary.find(id.url());
  if (it == schema_dictionary.end()) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "[Task Esimate Client] url: %s not found in schema dictionary",
      id.url().c_str());
    return;
  }

  value = it->second;
}

bool TaskEstimateClient::validate_json(
  const nlohmann::json & json,
  const nlohmann::json_schema::json_validator & validator,
  std::string & error) const
{
  try {
    validator.validate(json);
  } catch (const std::exception & e) {
    error = e.what();
    return false;
  }

  return true;
}

void TaskEstimateClient::validate_and_publish_estimate_request(
  const nlohmann::json & request_json,
  const nlohmann::json_schema::json_validator & validator,
  const std::string & request_id)
{

  // Get validation error if any
  std::string error;
  if (!validate_json(request_json, validator, error)) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "Error in request to [%s]: %s",
      request_id.c_str(),
      error.c_str());
    return;
  }

  // Check if request id is unique
  if (!this->request_id_set.insert(request_id).second) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "Error in request to [%s]: Request ID not unique!",
      request_id.c_str());
    return;
  }

  // Publish request
  this->request_publisher->publish(
    rmf_task_msgs::build<rmf_task_msgs::msg::ApiRequest>()
    .json_msg(request_json.dump())
    .request_id(request_id)
  );
}

std::string TaskEstimateClient::publish_request(
  const nlohmann::json & request_json)
{
  static const auto validator =
    make_validator(rmf_api_msgs::schemas::task_estimate_request);

  const std::string request_id = "estimate-task-" + std::to_string(count++);

  validate_and_publish_estimate_request(request_json, validator, request_id);
  return request_id;
}

// Response Handling
void TaskEstimateClient::validate_response(
  const nlohmann::json & response,
  const nlohmann::json_schema::json_validator & validator,
  std::string request_id)
{
  // Get validation error if any
  std::string error;
  if (!validate_json(response, validator, error)) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "Error in response to [%s]: %s",
      request_id.c_str(),
      error.c_str());
    return;
  }

  RCLCPP_INFO(
    _node->get_logger(),
    "Got Response to [%s]: %s",
    request_id.c_str(),
    response.dump().c_str());
}


void TaskEstimateClient::handle_response(
  const rmf_task_msgs::msg::ApiResponse::SharedPtr msg)
{
  if (request_id_set.find(msg->request_id) == request_id_set.end()) {
    return;
  }

  static const auto validator =
    make_validator(rmf_api_msgs::schemas::task_estimate_response);

  nlohmann::json response = nlohmann::json::parse(msg->json_msg);

  validate_response(
    response, validator, msg->request_id);

  response_map[msg->request_id] = response;
}

rclcpp::Node::SharedPtr TaskEstimateClient::node()
{
  return _node;
}

std::optional<nlohmann::json> TaskEstimateClient::get_response(
  const std::string & request_id)
{
  for (auto r = response_map.begin(); r != response_map.end(); r++) {
    if (r->first == request_id) {
      return r->second;
    }
  }

  return {};

}
