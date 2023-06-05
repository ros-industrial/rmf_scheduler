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

#include <thread>

#include "gtest/gtest.h"
#include "rmf_scheduler/system_time_executor.hpp"
#include "rmf_scheduler/system_time_utils.hpp"
#include "rmf_scheduler/test_utils.hpp"

class TestSystemTimeExecutor : public ::testing::Test
{
protected:
  void SetUp() override
  {
    using namespace rmf_scheduler;  // NOLINT(build/namespaces)
    actual_start_time_.clear();
    // Spin the executor
    thr_ = std::make_shared<std::thread>(
      [this]() {
        ste_.spin();
      });

    start_time_ = utils::now();

    // Generate events with random start time in the next 3s
    generated_events_ = test_utils::random_event_generator(
      utils::now(), utils::now() + 3e9, 200);

    // Add the event to the system time executor
    for (auto & event : generated_events_) {
      ste_.add_action(
        event, [event, this]() -> void
        {
          std::lock_guard<std::mutex> lk(mtx_);
          // Write actual start time
          actual_start_time_.emplace(event.id, utils::now());
        }
      );
    }
  }

  void TearDown() override
  {
    // stop the system time executor
    ste_.stop();
    thr_->join();
  }

  std::vector<rmf_scheduler::Event> generated_events_;

  rmf_scheduler::SystemTimeExecutor ste_;
  std::shared_ptr<std::thread> thr_;

  /// Map to record the actual start time
  std::unordered_map<std::string, uint64_t> actual_start_time_;
  std::mutex mtx_;

  /// Time when the system time executor started spinning
  uint64_t start_time_;
};

TEST_F(TestSystemTimeExecutor, ExecuteOnTime)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Wait for 4s. All events should be executed now.
  std::this_thread::sleep_for(std::chrono::seconds(4));

  // Examine and compare the scheduled start time and the actual start time
  std::cout << "Start time: " << start_time_ << std::endl;

  // std::cout << "id       "
  //   "\tsched start time\tactual start time\tlatency" << std::endl;

  double max = 0;
  double average = 0;
  double sq_average = 0;

  {  // Lock actual start time
    std::lock_guard<std::mutex> lk(mtx_);
    for (size_t i = 0; i < generated_events_.size(); i++) {
      const auto & event = generated_events_[i];
      double latency =
        test_utils::to_ms(actual_start_time_.at(event.id) - event.start_time);

      // Display execution time
      // std::cout << std::setprecision(8) <<
      //   event.id.substr(0, 8) << '\t' <<
      //   test_utils::to_ms(event.start_time - start_time_) << "ms\t\t" <<
      //   test_utils::to_ms(actual_start_time_.at(event.id) - start_time_) << "ms\t\t" <<
      //   latency << "ms\n";

      // Generate statistics
      if (latency > max) {
        max = latency;
      }
      average = (average * i + latency) / (i + 1);
      sq_average = (sq_average * i + latency * latency) / (i + 1);
    }
  }  // Unlock actual start time

  double deviation = sqrt(sq_average - average * average);

  std::cout << "Latency stats:\n";
  std::cout << "  max: " << max << "ms\n";
  std::cout << "  average: " << average << "ms\n";
  std::cout << "  standard deviation: " << deviation << "ms\n";

  // Execution latency should be smalled than 500ms
  EXPECT_LT(max, 500);
}


TEST_F(TestSystemTimeExecutor, ExecuteImmediately)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Create an immediate event,
  // start_time is NOW
  std::string event_id = "abcdefg-immediate";
  Event immediate_event {
    "Immediate test event",     // description
    "robot_task",               // type
    start_time_ - 1,                // start time
    0,                          // duration
    event_id,                   // id
    "",                         // series id
    ""                          // dag id
  };
  start_time_ = utils::now();

  // Add this event to the system time executor
  ste_.add_action(
    immediate_event, [event_id, this]() -> void
    {
      std::lock_guard<std::mutex> lk(mtx_);
      actual_start_time_.emplace(event_id, utils::now());
    }
  );

  // Sleep for 0.5s
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Check if the event is executed immediately
  {
    std::lock_guard<std::mutex> lk(mtx_);
    EXPECT_TRUE(actual_start_time_.find(event_id) != actual_start_time_.end());
    double latency =
      test_utils::to_ms(actual_start_time_.at(event_id) - start_time_);
    std::cout << "Immediate Task latency: " <<
      latency << "ms\n";
    EXPECT_LT(latency, 500);
  }
}

// Use a Burden event (the corresponding action doesn't exit immediately)
// This will test the mutex
// DON'T DO THIS IN ACTUAL USAGE
TEST_F(TestSystemTimeExecutor, BurdenEvent)
{
  using namespace rmf_scheduler;  // NOLINT(build/namespaces)

  // Wait for 0.5s. Some of the events should already be executed.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::string event_id = "abcdefg-burden";
  Event burden_event {
    "Burden event that last for 500ms",
    "robot_task",               // type
    utils::now() +
    static_cast<uint64_t>(1e9),    // start time
    0,                          // duration
    event_id,                   // id
    "",                         // series id
    ""                          // dag id
  };

  // Add this event to the system time executor
  ste_.add_action(
    burden_event, []() -> void
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  );

  // Wait for 1.5s. The burden event should be sleeping atm.
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  for (auto & event : generated_events_) {
    ste_.delete_action(event.id);
  }

  event_id = "abcdefg-addon";
  Event addon_event {
    "New event added after burden event",
    "robot_task",               // type
    utils::now() +
    static_cast<uint64_t>(1.5e9),    // start time
    0,                          // duration
    event_id,                   // id
    "",                         // series id
    ""                          // dag id
  };

  // Add this event to the system time executor
  ste_.add_action(
    addon_event, [event_id, this]() -> void
    {
      std::lock_guard<std::mutex> lk(mtx_);
      actual_start_time_.emplace(event_id, utils::now());
    }
  );

  // All events should be done by now, since all the others are cancelled
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  {
    std::lock_guard<std::mutex> lk(mtx_);
    std::cout << "Task executed after burden task:" << std::endl;
    for (auto & record : actual_start_time_) {
      if (record.second > start_time_ + 2.5e9) {
        std::cout << record.first << std::endl;
        EXPECT_EQ(record.first, event_id);
      }
    }
  }
}
