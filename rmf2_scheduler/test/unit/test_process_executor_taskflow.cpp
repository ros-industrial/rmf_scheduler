// Copyright 2023-2025 ROS Industrial Consortium Asia Pacific
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

#include <thread>

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "rmf2_scheduler/log.hpp"
#include "rmf2_scheduler/process_executor_taskflow.hpp"
#include "rmf2_scheduler/task_executor.hpp"

#include "sample_task_executors.hpp"


namespace rmf2_scheduler
{

}  // namespace rmf2_scheduler


class TestProcessExecutorTaskflow : public ::testing::Test
{
protected:
  rmf2_scheduler::data::Process::Ptr process;
  std::vector<rmf2_scheduler::data::Task::ConstPtr> tasks;
  std::string error;

  void SetUp() override
  {
    using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

    process = std::make_shared<data::Process>();
    process->id = "some_process";
    data::Graph & graph = process->graph;

    /*
    A -> B -> B1
    |     \
    |      -> B2 -> X
    |
    v
    C

    */
    graph.add_node("B2");
    graph.add_node("B1");
    graph.add_node("X");
    graph.add_node("C");
    graph.add_node("B");
    graph.add_node("A");
    graph.add_edge("A", "B");
    graph.add_edge("B", "B1");
    graph.add_edge("B", "B2");
    graph.add_edge("B2", "X");
    graph.add_edge("A", "C");

    auto task_type = "rmf2/test";

    tasks = {
      std::make_shared<data::Task>(
        "A", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status"),
      std::make_shared<data::Task>(
        "B", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status"),
      std::make_shared<data::Task>(
        "B1", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status"),
      std::make_shared<data::Task>(
        "B2", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status"),
      std::make_shared<data::Task>(
        "C", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status"),
      std::make_shared<data::Task>(
        "X", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status")
    };
  }

  void TearDown() override
  {
  }
};

TEST_F(TestProcessExecutorTaskflow, concurrency)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  rmf2_scheduler::log::setLogLevel(rmf2_scheduler::LogLevel::DEBUG);

  std::vector<int> concurrency_values = {1, 5};

  for (int concurrency : concurrency_values) {
    auto tem = std::make_shared<TaskExecutorManager>();
    auto tfpe = std::make_shared<TaskflowProcessExecutor>(tem, concurrency);

    rmf2_scheduler::TaskExecutor::Ptr
      test_task_executor = std::make_shared<rmf2_scheduler::TestTaskExecutor>();

    tem->load(
      "rmf2/test",
      test_task_executor
    );

    tfpe->run_async(process, tasks, error);
    tfpe->wait_for_all();
  }
}


TEST_F(TestProcessExecutorTaskflow, execute_in_order)
{
  rmf2_scheduler::log::setLogLevel(rmf2_scheduler::LogLevel::DEBUG);

  auto tem = std::make_shared<rmf2_scheduler::TaskExecutorManager>();

  auto tfpe = std::make_shared<rmf2_scheduler::TaskflowProcessExecutor>(tem, 1);

  rmf2_scheduler::TaskExecutor::Ptr
    test_task_executor = std::make_shared<rmf2_scheduler::TestTaskExecutor>();
  tem->load(
    "rmf2/test",
    test_task_executor
  );

  {
    using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
    auto scoped_process = std::make_shared<data::Process>();
    scoped_process->id = "some_process";
    data::Graph & graph = scoped_process->graph;

    /*
    A -> B -> B1
    |     \
    |      -> B2 -> X
    |
    v
    C

    */
    graph.add_node("B2");
    graph.add_node("B1");
    graph.add_node("X");
    graph.add_node("C");
    graph.add_node("B");
    graph.add_node("A");
    graph.add_edge("A", "B");
    graph.add_edge("B", "B1");
    graph.add_edge("B", "B2");
    graph.add_edge("B2", "X");
    graph.add_edge("A", "C");

    auto task_type = "rmf2/test";
    std::vector<rmf2_scheduler::data::Task::ConstPtr> scoped_tasks;
    scoped_tasks = {
      std::make_shared<data::Task>(
        "A", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status"),
      std::make_shared<data::Task>(
        "B", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status"),
      std::make_shared<data::Task>(
        "B1", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status"),
      std::make_shared<data::Task>(
        "B2", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status"),
      std::make_shared<data::Task>(
        "C", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status"),
      std::make_shared<data::Task>(
        "X", task_type, data::Time::from_localtime(
          "Dec 01 18:00:00 2024"), "dummy_status")
    };

    tfpe->run_async(scoped_process, scoped_tasks, error);
  }
  std::this_thread::sleep_for(std::chrono::seconds(1));
  tfpe->wait_for_all();
}

TEST_F(TestProcessExecutorTaskflow, failing_tasks)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  rmf2_scheduler::log::setLogLevel(rmf2_scheduler::LogLevel::DEBUG);

  auto tem = std::make_shared<TaskExecutorManager>();

  auto tfpe = std::make_shared<TaskflowProcessExecutor>(tem, 1);

  TaskExecutor::Ptr
    dummy_task_executor = std::make_shared<DummyTaskExecutor>();
  tem->load(
    "rmf2/test/dummytask",
    dummy_task_executor
  );
  TaskExecutor::Ptr
    failing_task_executor = std::make_shared<FailingTaskExecutor>();
  tem->load(
    "rmf2/test/failingtask",
    failing_task_executor
  );

  std::vector<rmf2_scheduler::data::Task::ConstPtr> failing_tasks;
  failing_tasks = {
    std::make_shared<data::Task>(
      "A", "rmf2/test/failingtask", data::Time::from_localtime(
        "Dec 01 18:00:00 2024"), "dummy_status"),
    std::make_shared<data::Task>(
      "B", "rmf2/test/dummytask", data::Time::from_localtime(
        "Dec 01 18:00:00 2024"), "dummy_status"),
    std::make_shared<data::Task>(
      "B1", "rmf2/test/dummytask", data::Time::from_localtime(
        "Dec 01 18:00:00 2024"), "dummy_status"),
    std::make_shared<data::Task>(
      "B2", "rmf2/test/dummytask", data::Time::from_localtime(
        "Dec 01 18:00:00 2024"), "dummy_status"),
    std::make_shared<data::Task>(
      "C", "rmf2/test/dummytask", data::Time::from_localtime(
        "Dec 01 18:00:00 2024"), "dummy_status"),
    std::make_shared<data::Task>(
      "X", "rmf2/test/dummytask", data::Time::from_localtime(
        "Dec 01 18:00:00 2024"), "dummy_status")
  };

  // Test that the workflow does not proceed after task A fails.
  // TODO(anyone): Test condition

  tfpe->run_async(process, failing_tasks, error);
  tfpe->wait_for_all();
}

TEST_F(TestProcessExecutorTaskflow, cancel_workflow)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  rmf2_scheduler::log::setLogLevel(rmf2_scheduler::LogLevel::DEBUG);

  auto tem = std::make_shared<TaskExecutorManager>();

  auto tfpe = std::make_shared<TaskflowProcessExecutor>(tem);

  rmf2_scheduler::TaskExecutor::Ptr
    test_task_executor = std::make_shared<rmf2_scheduler::TestTaskExecutor>();
  tem->load(
    "rmf2/test",
    test_task_executor
  );

  // Test workflow cancellation.
  // TODO(anyone): Test condition
  tfpe->run_async(process, tasks, error);

  // Cancel after some tasks have run.
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  auto fut = tfpe->cancel(process->id);
  fut.get();
  LOG_INFO("Cancel future stopped");
  tfpe->wait_for_all();
}
