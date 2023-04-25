#include "task_estimate_client_cpp/task_estimate_client.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task_estimate_client_node");
  TaskEstimateClientPtr client = TaskEstimateClient::make(node);
  nlohmann::json estimate_request;

  estimate_request["type"] = "estimate_task_request";
  estimate_request["fleet"] = "tinyRobot";
  estimate_request["robot"] = "tinyRobot1";

  nlohmann::json go_to_description;
  go_to_description["waypoint"] = "coe";
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

  estimate_request["request"]["task_request"] = task_request;


  std::vector<std::shared_future<nlohmann::json>> vector;

  for (int i = 0; i < 5; i++) {
    std::cout << "Sending estimate request" << std::endl;
    auto response_future = client->async_send_request(estimate_request);
    vector.push_back(response_future);
    std::cout << "Sent estimate request" << std::endl;
  }

  for (int i = 0; i < 5; i++) {
    // Check if optional has value and get future value
    if (!vector[i].valid()) {
      std::cout << "There is no future" << std::endl;
      return 1;
    }
    rclcpp::spin_until_future_complete(client->node(), vector[i]);
    auto response = vector[i].get();
    std::cout << "Got response: " << response.dump() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
