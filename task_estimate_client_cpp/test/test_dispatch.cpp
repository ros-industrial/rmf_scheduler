#include "task_estimate_client_cpp/task_estimate_client.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task_estimate_client_node");
  TaskEstimateClientPtr client = TaskEstimateClient::make(node);

  // Create robot task request
  nlohmann::json robot_task_request;
  robot_task_request["fleet"] = "tinyRobot";
  robot_task_request["robot"] = "tinyRobot1";
  robot_task_request["type"] = "robot_task_request";

  nlohmann::json go_to_description;
  go_to_description["waypoint"] = "lounge";
  go_to_description["orienation"] = 0.0;

  nlohmann::json go_to_activity;
  go_to_activity["category"] = "go_to_place";
  go_to_activity["description"] = go_to_description;

  nlohmann::json activity;
  activity["activity"] = go_to_activity;
  std::vector<nlohmann::json> phases = {activity};

  nlohmann::json task_request;
  task_request["category"] = "compose";
  task_request["unix_millis_earliest_start_time"] = 0;
  task_request["description"]["category"] = "go_to_place";
  task_request["description"]["phases"] = phases;

  robot_task_request["request"] = task_request;

  // Create dispatch task request
  nlohmann::json dispatch_task_request;
  dispatch_task_request["type"] = "dispatch_task_request";
  dispatch_task_request["request"] = task_request;

  // Create dispatch task request
  nlohmann::json estimate_request;
  estimate_request["type"] = "estimate_task_request";
  estimate_request["fleet"] = "tinyRobot";
  estimate_request["robot"] = "tinyRobot1";
  estimate_request["request"]["task_request"] = task_request;

  std::cout << "Sending dispatch requests" << std::endl;
  auto response_future = client->async_dispatch_request(robot_task_request);
  auto response_future_2 = client->async_dispatch_request(dispatch_task_request);
  std::cout << "Sent dispatch requests" << std::endl;

  // Check first response
  // Check if optional has value and get future value
  if (!response_future.valid()) {
    std::cout << "There is no future" << std::endl;
    return 1;
  }
  rclcpp::spin_until_future_complete(client->node(), response_future);
  auto response = response_future.get();
  std::cout << "Got response: " << response.dump() << std::endl;

  // Check second response
  if (!response_future_2.valid()) {
    std::cout << "There is no future" << std::endl;
    return 1;
  }
  rclcpp::spin_until_future_complete(client->node(), response_future_2);
  auto response_2 = response_future_2.get();
  std::cout << "Got response: " << response_2.dump() << std::endl;

  rclcpp::shutdown();
  return 0;
}
