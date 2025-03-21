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

#include <random>
#include <thread>

#include "gtest/gtest.h"
#include "rmf2_scheduler/system_time_executor.hpp"

namespace rmf2_scheduler
{

std::vector<SystemTimeAction>
random_action_generator(
  const data::Time & start_time,
  const data::Time & end_time,
  int sample_size)
{
  std::vector<SystemTimeAction> actions;
  actions.reserve(sample_size);
  std::default_random_engine gen;
  std::uniform_int_distribution<rs_time_point_value_t> dist(
    start_time.nanoseconds(),
    end_time.nanoseconds()
  );

  for (int i = 0; i < sample_size; i++) {
    data::Time start_time(dist(gen));
    SystemTimeAction action;
    action.time = start_time;
    actions.push_back(action);
  }
  return actions;
}

}  // namespace rmf2_scheduler

namespace
{

struct LatencyMetrics
{
  rmf2_scheduler::data::Duration max;
  rmf2_scheduler::data::Duration average;
  rmf2_scheduler::data::Duration standard_deviation;
};

}  // namespace

class TestSystemTimeExecutor : public ::testing::Test
{
protected:
  void SetUp() override
  {
    using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
    ste = SystemTimeExecutor::make_shared();
  }

  void TearDown() override
  {
    ste->stop();
    if (worker_thr_) {
      worker_thr_->join();
    }
  }

  const std::vector<rmf2_scheduler::SystemTimeAction> &
  initialize_actions(
    int sample_size,
    const rmf2_scheduler::data::Duration & duration
  )
  {
    using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

    actual_start_time.clear();
    actual_start_time.resize(sample_size);

    // generate action start time
    data::Time now(std::chrono::system_clock::now());
    actions_ = random_action_generator(
      now,
      now + duration,
      sample_size
    );

    // Set work function
    for (size_t i = 0; i < actions_.size(); i++) {
      actions_[i].work = [
        & actual_start_time = this->actual_start_time,
        i,
        & mtx = this->mtx_]() -> void {
          std::lock_guard<std::mutex> lk(mtx);
          actual_start_time[i] = std::chrono::system_clock::now();
        };
    }

    return actions_;
  }

  LatencyMetrics generate_latency_metrics()
  {
    using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

    LatencyMetrics metrics;
    rs_time_point_value_t sum = 0;
    rs_time_point_value_t sum_square = 0;
    size_t sample_size;

    metrics.max = data::Duration(0);

    {  // Lock
      std::lock_guard<std::mutex> lk(mtx_);
      sample_size = actions_.size();
      assert(actual_start_time.size() == sample_size);

      for (size_t i = 0; i < sample_size; i++) {
        data::Duration latency = actual_start_time[i] - actions_[i].time;

        sum += latency.nanoseconds();
        sum_square += latency.nanoseconds() * latency.nanoseconds();

        if (latency > metrics.max) {
          metrics.max = latency;
        }
      }
    }  // Unlock

    rs_time_point_value_t average = sum / sample_size;
    rs_time_point_value_t average_square = sum_square / sample_size;

    metrics.average = data::Duration(average);
    metrics.standard_deviation = data::Duration(std::sqrt(average_square - average * average));
    return metrics;
  }

  void print_latency_metrics(const LatencyMetrics & metrics)
  {
    std::cout << "Latency stats:\n";
    std::cout << "  max: " << metrics.max.seconds() * 1000 << "ms\n";
    std::cout << "  average: " << metrics.average.seconds() * 1000 << "ms\n";
    std::cout << "  standard deviation: " << metrics.standard_deviation.seconds() * 1000 << "ms\n";
  }

  void spin_and_detach(
    const rmf2_scheduler::SystemTimeExecutor::Ptr & ste
  )
  {
    // Spin the executor
    worker_thr_ = std::make_shared<std::thread>(
      [ste]() {
        ste->spin();
      });
  }

  rmf2_scheduler::SystemTimeExecutor::Ptr ste;
  std::vector<rmf2_scheduler::data::Time> actual_start_time;
  std::mutex mtx_;

private:
  std::vector<rmf2_scheduler::SystemTimeAction> actions_;

  /// Map to record the actual start time
  std::shared_ptr<std::thread> worker_thr_;
};

// Spin after everything is added
TEST_F(TestSystemTimeExecutor, execute_on_time)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  // Create 2000 actions within 4 seconds
  auto actions = initialize_actions(2000, data::Duration::from_seconds(4));

  // Add events to the system time executor
  for (auto & action : actions) {
    ste->add_action(action);
  }

  // start spinning
  spin_and_detach(ste);

  std::this_thread::sleep_for(std::chrono::seconds(5));

  auto metrics = generate_latency_metrics();

  print_latency_metrics(metrics);

  // Execution latency should be smalled than 500ms
  EXPECT_LT(metrics.max.seconds(), 0.5);
}

// Add actions as the execution is ongoing
TEST_F(TestSystemTimeExecutor, execute_on_time_otg)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  // Start spinning
  spin_and_detach(ste);

  // Create 200 actions within 4 seconds
  auto actions = initialize_actions(2000, data::Duration::from_seconds(4));

  for (auto & action : actions) {
    ste->add_action(action);
  }

  std::this_thread::sleep_for(std::chrono::seconds(5));

  auto metrics = generate_latency_metrics();

  print_latency_metrics(metrics);

  // Execution latency should be smalled than 500ms
  EXPECT_LT(metrics.max.seconds(), 0.5);
}

TEST_F(TestSystemTimeExecutor, ExecuteImmediately)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  // Start spinning
  spin_and_detach(ste);

  // Create 2000 actions within 4 seconds
  auto actions = initialize_actions(2000, data::Duration::from_seconds(4));

  for (auto & action : actions) {
    ste->add_action(action);
  }

  // Create an immediate action,
  // start_time is NOW
  bool completed = false;
  data::Duration latency;
  std::mutex mtx;
  SystemTimeAction action;
  action.time = std::chrono::system_clock::now();
  action.work = [&completed, &latency, &mtx, action]() {
      std::lock_guard<std::mutex> lk(mtx);
      completed = true;
      latency = data::Time(std::chrono::system_clock::now()) - action.time;
    };

  // Add this event to the system time executor
  ste->add_action(action);

  // Sleep for 0.5s
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Check if the event is executed immediately
  {
    std::lock_guard<std::mutex> lk(mtx);
    EXPECT_TRUE(completed);
    std::cout << "Immediate Task latency: " <<
      latency.seconds() * 1000 << "ms\n";
    EXPECT_LT(latency.seconds(), 0.5);
  }
}

// Use a Burden event (the corresponding action doesn't exit immediately)
// This will test the mutex
// DON'T DO THIS IN ACTUAL USAGE
TEST_F(TestSystemTimeExecutor, BurdenEvent)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  // Start spinning
  spin_and_detach(ste);

  // Create 2000 actions within 4 seconds
  auto actions = initialize_actions(2000, data::Duration::from_seconds(4));

  std::vector<std::string> action_ids;
  action_ids.reserve(2000);
  for (auto & action : actions) {
    std::string id = ste->add_action(action);
    action_ids.push_back(id);
  }

  // Wait for 0.5s. Some of the events should already be executed.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  bool completed = false;
  std::mutex mtx;
  SystemTimeAction burden_action;
  burden_action.time = std::chrono::system_clock::now() + std::chrono::seconds(1);
  burden_action.work = [&completed, &mtx, burden_action]() {
      std::this_thread::sleep_for(std::chrono::seconds(1));

      std::lock_guard<std::mutex> lk(mtx);
      completed = true;
    };

  // Add this event to the system time executor
  ste->add_action(burden_action);

  // Wait for 1.5s. The burden event should be sleeping atm.
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  for (auto & action_id : action_ids) {
    ste->delete_action(action_id);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  data::Time cutoff_time(std::chrono::system_clock::now());

  // All events should be done by now, since all the others are cancelled
  std::this_thread::sleep_for(std::chrono::milliseconds(600));

  {  // Lock
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto & record : actual_start_time) {
      EXPECT_LT(record, cutoff_time);
    }
  }  // Unlock

  {  // Lock
    std::lock_guard<std::mutex> lk(mtx);
    EXPECT_TRUE(completed);
  }  // Unlock
}


// delete action
TEST_F(TestSystemTimeExecutor, delete_action)
{
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  // Start spinning
  spin_and_detach(ste);

  // Create an action 1s from now
  SystemTimeAction action;
  std::atomic_bool action_completed = false;
  action.time = std::chrono::system_clock::now() + std::chrono::seconds(1);
  action.work = [&action_completed]() {
      action_completed = true;
    };

  // Add this event to the system time executor
  std::string id = ste->add_action(action);

  // Wait for 0.5s, cancel the action
  std::this_thread::sleep_for(std::chrono::milliseconds(990));

  ste->delete_action(id);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  EXPECT_FALSE(action_completed);
}
