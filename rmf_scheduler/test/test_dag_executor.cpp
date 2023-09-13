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

#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "rmf_scheduler/runtime/dag_executor.hpp"

class TestDAGExecutor : public ::testing::Test
{
protected:
  void SetUp() override
  {
    using namespace rmf_scheduler::data;  // NOLINT(build/namespaces)
    dag_description_ = {
      {"task1", {}},
      {"task2", {"task4", "task3"}},
      {"task3", {"task1"}},
      {"task4", {"task1"}}
    };
    dag_ = DAG(dag_description_);
  }

  void TearDown() override
  {
  }

  rmf_scheduler::data::DAG dag_;
  rmf_scheduler::data::DAG::Description dag_description_;
};

TEST_F(TestDAGExecutor, DAGExecutorBasic)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)
  std::vector<std::string> tasks_finished;
  std::mutex mtx;
  std::unordered_map<std::string, int> work_table = {
    {"task1", 1},
    {"task2", 2},
    {"task3", 3},
    {"task4", 4}
  };

  runtime::DAGExecutor dag_executor;

  // Create a generator that generate the function call based on task id
  runtime::DAGExecutor::WorkGenerator work_generator =
    [&tasks_finished, work_table, &mtx](const std::string & id)
    -> runtime::DAGExecutor::Work {
      int work_s = work_table.at(id);

      // The actual function called during the DAG execution
      auto work = [ =, &tasks_finished, &mtx]() {
          std::this_thread::sleep_for(std::chrono::seconds(work_s));

          // Lock thread before writing to tasks_finished
          std::lock_guard<std::mutex> lk(mtx);
          tasks_finished.push_back(id);
        };
      return work;
    };

  // Run a full execution
  auto future = dag_executor.run(dag_, work_generator);
  future.get();

  std::cout << "Order of tasks finished" << std::endl;
  for (auto & task : tasks_finished) {
    std::cout << task << std::endl;
  }

  EXPECT_EQ(tasks_finished.front(), "task1");
  EXPECT_EQ(tasks_finished.back(), "task2");
  EXPECT_EQ(static_cast<int>(tasks_finished.size()), 4);

  // Clear the history
  tasks_finished.clear();

  // Stop in the middle of the execution
  future = dag_executor.run(dag_, work_generator);

  // Sleep until the middle of task 2 being executed
  std::this_thread::sleep_for(std::chrono::seconds(3));
  dag_executor.cancel();

  future.get();

  for (auto & task : tasks_finished) {
    std::cout << task << std::endl;
  }
}
