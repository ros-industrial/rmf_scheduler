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

#include <thread>

#include "rmf2_scheduler/data/uuid.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"
#include "rmf2_scheduler/executor_data.hpp"
#include "rmf2_scheduler/log.hpp"
#include "rmf2_scheduler/scheduler.hpp"

namespace rmf2_scheduler
{

class TestTaskExecutor : public TaskExecutor
{
public:
  TestTaskExecutor() = default;
  ~TestTaskExecutor()
  {
    if (thr) {
      thr->join();
    }
  }
  bool build(
    const data::Task::ConstPtr & task,
    ExecutorData * data,
    std::string & error
  ) override
  {
    if (!task->duration.has_value()) {
      error = "No duration set, cannot execute";
      return false;
    }

    data::Duration duration(std::chrono::seconds(20));
    data->set_data_as_json(nlohmann::json(duration));
    return true;
  }

  bool start(
    const std::string & id,
    const ExecutorData * data,
    std::string & error
  ) override
  {
    auto duration_j = data->get_data_as_json();
    if (!duration_j.has_value()) {
      error = "Corrupted data!!";
      return false;
    }

    data::Duration duration = data->get_data_as_json()->get<data::Duration>();

    thr = std::make_shared<std::thread>(
      [this, id, duration]() {
        std::this_thread::sleep_for(duration.to_chrono<std::chrono::seconds>());
        ExecutorData completion_data;
        completion_data.set_data_as_c_string("Hello World!!");
        notify_completion(
          id, true, &completion_data
        );
      }
    );

    return true;
  }

private:
  std::shared_ptr<std::thread> thr;
};

}  // namespace rmf2_scheduler

int main()
{
  // rmf2_scheduler::log::setLogLevel(rmf2_scheduler::LogLevel::DEBUG);
  auto ste = rmf2_scheduler::SystemTimeExecutor::make_shared();
  auto tem = rmf2_scheduler::TaskExecutorManager::make_shared();

  rmf2_scheduler::TaskExecutor::Ptr
    test_task_executor = std::make_shared<rmf2_scheduler::TestTaskExecutor>();
  tem->load(
    "rmf2/test",
    test_task_executor
  );

  auto scheduler = rmf2_scheduler::Scheduler::make_shared(
    rmf2_scheduler::SchedulerOptions::make_shared(),
    rmf2_scheduler::storage::ScheduleStream::create_default(
      "http://localhost:9090/ngsi-ld"
    ),
    ste,
    nullptr,
    nullptr,
    nullptr,
    tem
  );

  std::thread spin_thr(&rmf2_scheduler::SystemTimeExecutor::spin, ste);

  // Add task
  auto task = rmf2_scheduler::data::Task::make_shared();
  task->id = rmf2_scheduler::data::gen_uuid();
  task->type = "rmf2/test";
  task->start_time = std::chrono::system_clock::now();
  task->duration = std::chrono::seconds(20);

  std::string error;
  scheduler->perform(
    rmf2_scheduler::cache::Action::create(
      rmf2_scheduler::data::action_type::TASK_ADD,
      rmf2_scheduler::cache::ActionPayload().task(task)
    ),
    error
  );

  spin_thr.join();
  return 0;
}
