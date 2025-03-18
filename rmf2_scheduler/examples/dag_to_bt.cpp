// Copyright 2024 ROS Industrial Consortium Asia Pacific
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

#include <iostream>

#include "SimpleAmqpClient/SimpleAmqpClient.h"

#include "rmf2_scheduler/data/process.hpp"
#include "rmf2_scheduler/data/task.hpp"
#include "rmf2_scheduler/data/uuid.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"
#include "rmf2_scheduler/utils/dag_helper.hpp"
#include "rmf2_scheduler/utils/tree_converter.hpp"

int main()
{
  using Process = rmf2_scheduler::data::Process;
  using Event = rmf2_scheduler::data::Event;
  using Task = rmf2_scheduler::data::Task;
  using Time = rmf2_scheduler::data::Time;
  using Duration = rmf2_scheduler::data::Duration;

  const std::string go_to_place_task_type = "rmf2/go_to_place";
  const std::string robot1_id = "R1";
  const std::string robot2_id = "R2";
  const std::string robot3_id = "R3";
  const std::string robot4_id = "R4";

  // Task 1 R1 -> Pickup 1
  Task::Ptr task1 = std::make_shared<Task>();
  task1->id = rmf2_scheduler::data::gen_uuid();
  task1->type = go_to_place_task_type;
  task1->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task1->status = "DRAFT";
  task1->description = "R1 -> Pickup 1";
  task1->duration = Duration::from_seconds(600);
  task1->resource_id = robot1_id;
  task1->task_details = R"({
    "end_location": "pickup1"
  })"_json;

  // Task 2 R2 -> Pickup 1
  Task::Ptr task2 = std::make_shared<Task>();
  task2->id = rmf2_scheduler::data::gen_uuid();
  task2->type = go_to_place_task_type;
  task2->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task2->status = "DRAFT";
  task2->description = "R2 -> Pickup 1";
  task2->duration = Duration::from_seconds(600);
  task2->resource_id = robot2_id;
  task2->task_details = R"({
    "end_location": "pickup1"
  })"_json;

  // Task 3 R1 -> Drop 1
  Task::Ptr task3 = std::make_shared<Task>();
  task3->id = rmf2_scheduler::data::gen_uuid();
  task3->type = go_to_place_task_type;
  task3->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task3->status = "DRAFT";
  task3->description = "R1 -> Drop 1";
  task3->duration = Duration::from_seconds(600);
  task3->resource_id = robot1_id;
  task3->task_details = R"({
    "end_location": "drop1"
  })"_json;

  // Task 4 R3 -> Pickup 1
  Task::Ptr task4 = std::make_shared<Task>();
  task4->id = rmf2_scheduler::data::gen_uuid();
  task4->type = go_to_place_task_type;
  task4->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task4->status = "DRAFT";
  task4->description = "R3 -> Pickup 1";
  task4->duration = Duration::from_seconds(600);
  task4->resource_id = robot3_id;
  task4->task_details = R"({
    "end_location": "pickup1"
  })"_json;

  // Task 5 R2 -> Drop 1
  Task::Ptr task5 = std::make_shared<Task>();
  task5->id = rmf2_scheduler::data::gen_uuid();
  task5->type = go_to_place_task_type;
  task5->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task5->status = "DRAFT";
  task5->description = "R2 -> Drop 1";
  task5->duration = Duration::from_seconds(600);
  task5->resource_id = robot2_id;
  task5->task_details = R"({
    "end_location": "drop1"
  })"_json;

  // Task 6 R1 -> Resting 1
  Task::Ptr task6 = std::make_shared<Task>();
  task6->id = rmf2_scheduler::data::gen_uuid();
  task6->type = go_to_place_task_type;
  task6->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task6->status = "DRAFT";
  task6->description = "R1 -> Resting 1";
  task6->duration = Duration::from_seconds(600);
  task6->resource_id = robot1_id;
  task6->task_details = R"({
    "end_location": "resting1"
  })"_json;

  // Task 7 R4 -> Pickup 1
  Task::Ptr task7 = std::make_shared<Task>();
  task7->id = rmf2_scheduler::data::gen_uuid();
  task7->type = go_to_place_task_type;
  task7->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task7->status = "DRAFT";
  task7->description = "R4 -> Pickup 1";
  task7->duration = Duration::from_seconds(600);
  task7->resource_id = robot4_id;
  task7->task_details = R"({
    "end_location": "pickup1"
  })"_json;

  // Task 8 R3 -> Drop 1
  Task::Ptr task8 = std::make_shared<Task>();
  task8->id = rmf2_scheduler::data::gen_uuid();
  task8->type = go_to_place_task_type;
  task8->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task8->status = "DRAFT";
  task8->description = "R3 -> Drop 1";
  task8->duration = Duration::from_seconds(600);
  task8->resource_id = robot3_id;
  task8->task_details = R"({
    "end_location": "drop1"
  })"_json;

  // Task 9 R2 -> Rest 2
  Task::Ptr task9 = std::make_shared<Task>();
  task9->id = rmf2_scheduler::data::gen_uuid();
  task9->type = go_to_place_task_type;
  task9->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task9->status = "DRAFT";
  task9->description = "R2 -> Rest 2";
  task9->duration = Duration::from_seconds(600);
  task9->resource_id = robot2_id;
  task9->task_details = R"({
    "end_location": "rest2"
  })"_json;

  // Task 10 R4 -> Drop 1
  Task::Ptr task10 = std::make_shared<Task>();
  task10->id = rmf2_scheduler::data::gen_uuid();
  task10->type = go_to_place_task_type;
  task10->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task10->status = "DRAFT";
  task10->description = "R4 -> Drop 1";
  task10->duration = Duration::from_seconds(600);
  task10->resource_id = robot4_id;
  task10->task_details = R"({
    "end_location": "drop1"
  })"_json;

  // Task 11 R3 -> Rest 3
  Task::Ptr task11 = std::make_shared<Task>();
  task11->id = rmf2_scheduler::data::gen_uuid();
  task11->type = go_to_place_task_type;
  task11->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task11->status = "DRAFT";
  task11->description = "R3 -> Rest 3";
  task11->duration = Duration::from_seconds(600);
  task11->resource_id = robot3_id;
  task11->task_details = R"({
    "end_location": "rest3"
  })"_json;

  // Task 12 R4 -> Rest 4
  Task::Ptr task12 = std::make_shared<Task>();
  task12->id = rmf2_scheduler::data::gen_uuid();
  task12->type = go_to_place_task_type;
  task12->start_time = Time::from_localtime("Dec 01 18:00:00 2024");
  task12->status = "DRAFT";
  task12->description = "R4 -> Rest 4";
  task12->duration = Duration::from_seconds(600);
  task12->resource_id = robot4_id;
  task12->task_details = R"({
    "end_location": "rest4"
  })"_json;
  std::unordered_map<std::string, Event::Ptr> event_map = {
    {task1->id, task1},
    {task2->id, task2},
    {task3->id, task3},
    {task4->id, task4},
    {task5->id, task5},
    {task6->id, task6},
    {task7->id, task7},
    {task8->id, task8},
    {task9->id, task9},
    {task10->id, task10},
    {task11->id, task11},
    {task12->id, task12},
  };

  Process::Ptr process = std::make_unique<Process>();
  process->id = rmf2_scheduler::data::gen_uuid();

  // Add all nodes
  for (auto & itr : event_map) {
    process->graph.add_node(itr.second->id);
  }

  process->graph.add_edge(task1->id, task2->id);
  process->graph.add_edge(task1->id, task3->id);
  process->graph.add_edge(task2->id, task4->id);
  process->graph.add_edge(task2->id, task5->id);
  process->graph.add_edge(task3->id, task5->id);
  process->graph.add_edge(task3->id, task6->id);
  process->graph.add_edge(task4->id, task7->id);
  process->graph.add_edge(task4->id, task8->id);
  process->graph.add_edge(task5->id, task8->id);
  process->graph.add_edge(task5->id, task9->id);
  process->graph.add_edge(task7->id, task10->id);
  process->graph.add_edge(task8->id, task10->id);
  process->graph.add_edge(task8->id, task11->id);
  process->graph.add_edge(task10->id, task12->id);

  std::ostringstream oss;
  process->graph.dump(oss);
  std::cout << oss.str() << std::endl;

  // Update event start time
  rmf2_scheduler::utils::update_dag_start_time(
    process->graph, event_map
  );

  nlohmann::json j;
  j["process"] = nlohmann::json(*process);
  j["events"] = nlohmann::json(
    std::vector<Task>{
    *task1, *task2, *task3, *task4, *task5, *task6,
    *task7, *task8, *task9, *task10, *task11, *task12
  }
  );

  std::cout << j.dump(2) << std::endl;

  rmf2_scheduler::utils::TreeConversion tree_converter;
  auto result = tree_converter.convert_to_tree(
    process->graph,
    [ = ](const std::string & id) {
      Task::Ptr task = std::dynamic_pointer_cast<Task>(event_map.find(id)->second);
      std::ostringstream oss;
      oss << "SubTree "
          << "ID=\"ReplaceMAPF\" "
          << "task_id=\"urn:" << task->id << "\" "
          << "bt_id=\"{bt_id}\" "
          << "task_status=\"{task_status}\" "
          << "coordinates=\"" << task->task_details["end_location"] << "\" "
          << "connection=\"{connection}\" "
          << "asset_name=\"" << *task->resource_id << "\""
      ;
      return oss.str();
    }
  );
  std::cout << result << std::endl;

  // Sent through AMQP
  nlohmann::json obj;
  obj["id"] = "urn:" + rmf2_scheduler::data::gen_uuid();
  obj["type"] = "Schedule";
  obj["scheduleType"] = "xml";
  obj["taskTime"] = "";
  obj["payload"] = result;

  std::string obj_str = obj.dump();
  std::string queue_name = "@RECEIVE@-event_mgr";
  std::string exchange_name = "@RECEIVE@";
  std::string routing_key = "";
  AmqpClient::Channel::ptr_t connection = AmqpClient::Channel::Create("localhost", 5672);
  connection->BindQueue(queue_name, exchange_name, routing_key);

  auto message = AmqpClient::BasicMessage::Create(obj_str);
  message->ContentType("application/json");

  try {
    connection->BasicPublish(exchange_name, routing_key, message, false);
  } catch (const std::exception & e) {
    std::cerr << "Failed to publish message: " << e.what() << std::endl;
  }
}
