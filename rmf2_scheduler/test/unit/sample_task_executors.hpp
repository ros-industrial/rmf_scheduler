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

#ifndef UNIT__SAMPLE_TASK_EXECUTORS_HPP_
#define UNIT__SAMPLE_TASK_EXECUTORS_HPP_

#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rmf2_scheduler/data/json_serializer.hpp"

#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "rmf2_scheduler/task_executor.hpp"
#include "rmf2_scheduler/executor_data.hpp"

namespace rmf2_scheduler
{

class FailingTaskExecutor : public TaskExecutor
{
public:
  FailingTaskExecutor() = default;
  bool build(
    const data::Task::ConstPtr & task,
    ExecutorData * data,
    std::string & /*error*/
  ) override
  {
    // Check task and data are valid
    assert(task);
    assert(data);

    // Fail as expected
    data->set_data_as_string("This was never going to work.");
    return true;
  }

  bool start(
    const std::string & id,
    const ExecutorData * data,
    std::string & error
  ) override
  {
    // Check data is valid
    assert(data);

    EXPECT_EQ(data->get_data_as_string(), "This was never going to work.");

    std::ostringstream oss;
    oss << "Task " << id << " failed.";
    error = oss.str();
    return false;
  }
};

class DummyTaskExecutor : public TaskExecutor
{
public:
  DummyTaskExecutor() = default;
  bool build(
    const data::Task::ConstPtr & task,
    ExecutorData * data,
    std::string & /*error*/
  ) override
  {
    // Check task and data are valid
    assert(task);
    assert(data);
    return true;
  }

  bool start(
    const std::string & id,
    const ExecutorData * data,
    std::string & /*error*/
  ) override
  {
    // Check data is valid
    assert(data);
    ExecutorData completion_data;
    completion_data.set_data_as_c_string("Hello World!!");
    notify_completion(
      id, true, &completion_data
    );
    return true;
  }
};


class TestTaskExecutor : public TaskExecutor
{
public:
  TestTaskExecutor() = default;
  ~TestTaskExecutor()
  {
    for (auto & thr : threads) {
      if (thr.joinable()) {
        thr.join();
      }
    }
  }
  bool build(
    const data::Task::ConstPtr & task,
    ExecutorData * data,
    std::string & /*error*/
  ) override
  {
    // Check task and data are valid
    assert(task);
    assert(data);

    data::Duration duration(std::chrono::seconds(1));
    if (task->duration.has_value()) {
      duration = task->duration.value();
    }

    data->set_data_as_json(nlohmann::json(duration));
    return true;
  }

  bool start(
    const std::string & id,
    const ExecutorData * data,
    std::string & error
  ) override
  {
    // Check data is valid
    assert(data);
    auto duration_j = data->get_data_as_json();
    if (!duration_j.has_value()) {
      error = "Corrupted data!!";
      return false;
    }

    data::Duration duration = data->get_data_as_json()->get<data::Duration>();

    std::lock_guard lk(mtx);
    threads.push_back(
      std::thread(
        [this, id, duration]()
        {
          std::this_thread::sleep_for(duration.to_chrono<std::chrono::seconds>());

          ExecutorData completion_data;
          completion_data.set_data_as_c_string("Hello World!!");
          notify_completion(
            id, true, &completion_data
          );
        }
    ));

    return true;
  }

private:
  std::vector<std::thread> threads;
  std::mutex mtx;
};

}  // namespace rmf2_scheduler

#endif  // UNIT__SAMPLE_TASK_EXECUTORS_HPP_
