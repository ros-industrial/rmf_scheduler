// Copyright 2025 ROS Industrial Consortium Asia Pacific
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

#include <filesystem>
#include <fstream>

#include "gtest/gtest.h"

#include "rmf2_scheduler/data/uuid.hpp"
#include "rmf2_scheduler/cache/action.hpp"
#include "rmf2_scheduler/storage/schedule_stream_simple.hpp"

namespace fs = std::filesystem;

namespace
{

static constexpr char MANIFEST_FILENAME[] = "manifest.r2ts";
static constexpr char BACKUP_FILENAME_PREFIX[] = "backup-";
static constexpr char BACKUP_FILENAME_EXTENSION[] = ".r2tsc";

std::string generate_backup_folder_path()
{
  std::string test_resources_path = TEST_RESOURCES_DIRECTORY;
  return test_resources_path + "/simple_storage/tmp-" + rmf2_scheduler::data::gen_uuid();
}

std::string get_sample_folder_path(
  const std::string & folder_name
)
{
  std::string test_resources_path = TEST_RESOURCES_DIRECTORY;
  return test_resources_path + "/simple_storage/sample-" + folder_name;
}

bool compare_backups(
  const std::string & path_1,
  const std::string & path_2
)
{
  // Check manifests exist
  fs::path backup_folder_filepath_1(path_1);
  fs::path manifest_filepath_1 = backup_folder_filepath_1 / MANIFEST_FILENAME;

  fs::path backup_folder_filepath_2(path_2);
  fs::path manifest_filepath_2 = backup_folder_filepath_2 / MANIFEST_FILENAME;

  if (!fs::exists(manifest_filepath_1)) {
    std::cerr << "Backup path [" << path_1
              << "] to compare is invalid, manifest file doesn't exist." << std::endl;
    return false;
  }

  if (!fs::exists(manifest_filepath_2)) {
    std::cerr << "Backup path [" << path_2
              << "] to compare is invalid, manifest file doesn't exist." << std::endl;
    return false;
  }
  std::ifstream manifest_rf1(manifest_filepath_1, std::ios::ate | std::ios::binary);
  std::ifstream manifest_rf2(manifest_filepath_2, std::ios::ate | std::ios::binary);

  if (manifest_rf1.fail() || manifest_rf2.fail()) {
    std::cerr << "Unable to open backup manifest file for comparison." << std::endl;
    return false;  // file problem
  }

  if (manifest_rf1.tellg() != manifest_rf2.tellg()) {
    std::cerr << "Backup manifest has different number of backups. "
              << "Byte size [" << manifest_rf1.tellg() << " != " << manifest_rf2.tellg()
              << "]." << std::endl;
    return false;  // size mismatch
  }

  // Retrieve the backup filepaths
  size_t backup_size = manifest_rf1.tellg() / 36;
  std::vector<std::string> backup_ids_1;
  std::vector<std::string> backup_ids_2;
  backup_ids_1.reserve(backup_size);
  backup_ids_2.reserve(backup_size);

  manifest_rf1.seekg(0, std::ifstream::beg);
  manifest_rf2.seekg(0, std::ifstream::beg);

  char uuid_c_str[36];
  for (size_t i = 0; i < backup_size; i++) {
    manifest_rf1.read(uuid_c_str, 36);
    std::string uuid_1(uuid_c_str, 36);
    backup_ids_1.push_back(uuid_1);

    manifest_rf2.read(uuid_c_str, 36);
    std::string uuid_2(uuid_c_str, 36);
    backup_ids_2.push_back(uuid_2);
  }

  manifest_rf1.close();
  manifest_rf2.close();

  // Compare the backup files
  for (size_t i = 0; i < backup_size; i++) {
    // Retrieve the backup filenames to compare
    std::string backup_filename_1 =
      std::string(BACKUP_FILENAME_PREFIX) + backup_ids_1[i] + BACKUP_FILENAME_EXTENSION;

    std::string backup_filename_2 =
      std::string(BACKUP_FILENAME_PREFIX) + backup_ids_2[i] + BACKUP_FILENAME_EXTENSION;

    fs::path backup_filepath_1 = backup_folder_filepath_1 / backup_filename_1;
    fs::path backup_filepath_2 = backup_folder_filepath_2 / backup_filename_2;
    std::ifstream backup_rf_1(backup_filepath_1, std::ios::ate | std::ios::binary);
    std::ifstream backup_rf_2(backup_filepath_2, std::ios::ate | std::ios::binary);
    backup_rf_1.seekg(0, std::ifstream::beg);
    backup_rf_2.seekg(0, std::ifstream::beg);

    bool result = std::equal(
      std::istreambuf_iterator<char>(backup_rf_1.rdbuf()),
      std::istreambuf_iterator<char>(),
      std::istreambuf_iterator<char>(backup_rf_2.rdbuf())
    );

    if (!result) {
      std::cerr << "Backup files doesn't match ["
                << backup_filename_1 << "] and ["
                << backup_filename_2 << "]." << std::endl;

      return false;
    }
  }
  return true;
}

}  // namespace

class TestStorageScheduleStreamSimple : public ::testing::Test
{
};

TEST_F(TestStorageScheduleStreamSimple, read_schedule_empty) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::storage;  // NOLINT(build/namespaces)

  ScheduleStream::Ptr schedule_stream = std::make_shared<simple::ScheduleStream>(
    5,
    generate_backup_folder_path()
  );

  auto schedule_cache = ScheduleCache::make_shared();
  TimeWindow time_window;
  time_window.start = Time(0);
  time_window.end = Time::max();
  std::string error;

  bool result = schedule_stream->read_schedule(
    schedule_cache,
    time_window,
    error
  );

  EXPECT_TRUE(result);
  EXPECT_TRUE(schedule_cache->get_all_tasks().empty());
}

TEST_F(TestStorageScheduleStreamSimple, read_schedule_event_only) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::storage;  // NOLINT(build/namespaces)

  ScheduleStream::Ptr schedule_stream = std::make_shared<simple::ScheduleStream>(
    5,
    get_sample_folder_path("read-schedule-event-only")
  );

  auto schedule_cache = ScheduleCache::make_shared();
  TimeWindow time_window;
  time_window.start = Time(0);
  time_window.end = Time::max();
  std::string error;

  bool result = schedule_stream->read_schedule(
    schedule_cache,
    time_window,
    error
  );

  EXPECT_TRUE(result);
  ASSERT_TRUE(schedule_cache->has_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"));
  auto task = schedule_cache->get_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  EXPECT_EQ(task->type, "task_type");
  EXPECT_EQ(task->duration, Duration(3600, 0));
}

TEST_F(TestStorageScheduleStreamSimple, read_schedule_event_and_process) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::storage;  // NOLINT(build/namespaces)

  ScheduleStream::Ptr schedule_stream = std::make_shared<simple::ScheduleStream>(
    5,
    get_sample_folder_path("read-schedule-event-and-process")
  );

  auto schedule_cache = ScheduleCache::make_shared();
  TimeWindow time_window;
  time_window.start = Time(0);
  time_window.end = Time::max();
  std::string error;

  bool result = schedule_stream->read_schedule(
    schedule_cache,
    time_window,
    error
  );

  EXPECT_TRUE(result);

  // Check task
  ASSERT_TRUE(schedule_cache->has_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"));
  auto task1 = schedule_cache->get_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  ASSERT_TRUE(task1->process_id.has_value());
  EXPECT_EQ(task1->process_id.value(), "13aa1c62-64ca-495d-a4b7-84de6a00f56a");

  ASSERT_TRUE(schedule_cache->has_task("8a9f4819-bbd4-4550-acce-e061d3289973"));
  auto task2 = schedule_cache->get_task("8a9f4819-bbd4-4550-acce-e061d3289973");
  ASSERT_TRUE(task2->process_id.has_value());
  EXPECT_EQ(task2->process_id.value(), "13aa1c62-64ca-495d-a4b7-84de6a00f56a");

  // Check process
  ASSERT_TRUE(schedule_cache->has_process("13aa1c62-64ca-495d-a4b7-84de6a00f56a"));
  auto process = schedule_cache->get_process("13aa1c62-64ca-495d-a4b7-84de6a00f56a");
  EXPECT_TRUE(process->graph.has_node("8a9f4819-bbd4-4550-acce-e061d3289973"));
  auto node = process->graph.get_node("8a9f4819-bbd4-4550-acce-e061d3289973");
  EXPECT_EQ(Edge("hard"), node->inbound_edges().at("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"));
}

TEST_F(
  TestStorageScheduleStreamSimple,
  read_schedule_event_and_process_with_extra
) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::storage;  // NOLINT(build/namespaces)

  ScheduleStream::Ptr schedule_stream = std::make_shared<simple::ScheduleStream>(
    5,
    get_sample_folder_path("read-schedule-event-and-process")
  );

  auto schedule_cache = ScheduleCache::make_shared();
  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-22T01:15:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T03:15:00Z");
  std::string error;

  bool result = schedule_stream->read_schedule(
    schedule_cache,
    time_window,
    error
  );

  EXPECT_TRUE(result);

  // Check task
  ASSERT_TRUE(schedule_cache->has_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"));
  auto task1 = schedule_cache->get_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  ASSERT_TRUE(task1->process_id.has_value());
  EXPECT_EQ(task1->process_id.value(), "13aa1c62-64ca-495d-a4b7-84de6a00f56a");

  ASSERT_TRUE(schedule_cache->has_task("8a9f4819-bbd4-4550-acce-e061d3289973"));
  auto task2 = schedule_cache->get_task("8a9f4819-bbd4-4550-acce-e061d3289973");
  ASSERT_TRUE(task2->process_id.has_value());
  EXPECT_EQ(task2->process_id.value(), "13aa1c62-64ca-495d-a4b7-84de6a00f56a");

  // Check process
  ASSERT_TRUE(schedule_cache->has_process("13aa1c62-64ca-495d-a4b7-84de6a00f56a"));
  auto process = schedule_cache->get_process("13aa1c62-64ca-495d-a4b7-84de6a00f56a");
  EXPECT_TRUE(process->graph.has_node("8a9f4819-bbd4-4550-acce-e061d3289973"));
  auto node = process->graph.get_node("8a9f4819-bbd4-4550-acce-e061d3289973");
  EXPECT_EQ(Edge("hard"), node->inbound_edges().at("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"));
}

TEST_F(TestStorageScheduleStreamSimple, write_schedule_time_window) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::storage;  // NOLINT(build/namespaces)

  std::string backup_folder_path = generate_backup_folder_path();
  ScheduleStream::Ptr schedule_stream = std::make_shared<simple::ScheduleStream>(
    5,
    backup_folder_path
  );

  auto schedule_cache = ScheduleCache::make_shared();

  // Add tasks and process
  Task::Ptr task1 = std::make_shared<Task>();
  task1->id = "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6";
  task1->type = "task_type";
  task1->start_time = Time::from_ISOtime("2025-01-22T02:15:00Z");
  task1->status = "ongoing";

  Task::Ptr task2 = std::make_shared<Task>();
  task2->id = "8a9f4819-bbd4-4550-acce-e061d3289973";
  task2->type = "task_type";
  task2->start_time = Time::from_ISOtime("2025-01-22T04:15:00Z");
  task2->status = "ongoing";

  Task::Ptr task3 = std::make_shared<Task>();
  task3->id = "ad0b5ee3-fd53-489b-a712-99d8b24161e8";
  task3->type = "task_type";
  task3->start_time = Time::from_ISOtime("2025-01-24T04:15:00Z");  // not in range
  task3->status = "ongoing";

  std::vector<Task::Ptr> tasks {task1, task2, task3};

  Process::Ptr process = std::make_shared<Process>();
  process->id = "13aa1c62-64ca-495d-a4b7-84de6a00f56a";
  process->graph.add_node("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  process->graph.add_node("8a9f4819-bbd4-4550-acce-e061d3289973");
  process->graph.add_edge(
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "8a9f4819-bbd4-4550-acce-e061d3289973"
  );

  std::vector<Process::Ptr> processes{process};
  std::string error;

  for (auto & task : tasks) {
    auto cache_action = Action::create(
      action_type::TASK_ADD,
      ActionPayload().task(task)
    );

    ASSERT_TRUE(cache_action->validate(schedule_cache, error));
    cache_action->apply();
  }

  for (auto & process : processes) {
    auto cache_action = Action::create(
      action_type::PROCESS_ADD,
      ActionPayload().process(process)
    );

    EXPECT_TRUE(cache_action->validate(schedule_cache, error));
    cache_action->apply();
  }

  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-21T10:00:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T10:00:00Z");

  EXPECT_TRUE(
    schedule_stream->write_schedule(
      schedule_cache,
      time_window,
      error
  ));

  // Check newly generated schedule matches the sample
  EXPECT_TRUE(
    compare_backups(
      backup_folder_path,
      get_sample_folder_path("write-schedule-time-window")
    )
  );
}

TEST_F(TestStorageScheduleStreamSimple, write_schedule_record) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::storage;  // NOLINT(build/namespaces)

  std::string backup_folder_path = generate_backup_folder_path();
  ScheduleStream::Ptr schedule_stream = std::make_shared<simple::ScheduleStream>(
    5,
    backup_folder_path
  );

  auto schedule_cache = ScheduleCache::make_shared();

  // Add tasks and process
  Task::Ptr task1 = std::make_shared<Task>();
  task1->id = "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6";
  task1->type = "task_type";
  task1->start_time = Time::from_ISOtime("2025-01-22T02:15:00Z");
  task1->status = "ongoing";

  Task::Ptr task2 = std::make_shared<Task>();
  task2->id = "8a9f4819-bbd4-4550-acce-e061d3289973";
  task2->type = "task_type";
  task2->start_time = Time::from_ISOtime("2025-01-22T04:15:00Z");
  task2->status = "ongoing";

  Task::Ptr task3 = std::make_shared<Task>();
  task3->id = "ad0b5ee3-fd53-489b-a712-99d8b24161e8";
  task3->type = "task_type";
  task3->start_time = Time::from_ISOtime("2025-01-24T04:15:00Z");  // not in range
  task3->status = "ongoing";

  std::vector<Task::Ptr> tasks {task1, task2, task3};

  Process::Ptr process = std::make_shared<Process>();
  process->id = "13aa1c62-64ca-495d-a4b7-84de6a00f56a";
  process->graph.add_node("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  process->graph.add_node("8a9f4819-bbd4-4550-acce-e061d3289973");
  process->graph.add_edge(
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "8a9f4819-bbd4-4550-acce-e061d3289973"
  );

  std::vector<Process::Ptr> processes{process};
  std::vector<ScheduleChangeRecord> records;
  std::string error;

  for (auto & task : tasks) {
    auto cache_action = Action::create(
      action_type::TASK_ADD,
      ActionPayload().task(task)
    );

    ASSERT_TRUE(cache_action->validate(schedule_cache, error));
    cache_action->apply();
    records.push_back(cache_action->record());
  }

  for (auto & process : processes) {
    auto cache_action = Action::create(
      action_type::PROCESS_ADD,
      ActionPayload().process(process)
    );

    EXPECT_TRUE(cache_action->validate(schedule_cache, error));
    cache_action->apply();
    records.push_back(cache_action->record());
  }

  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-21T10:00:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T10:00:00Z");

  EXPECT_TRUE(
    schedule_stream->write_schedule(
      schedule_cache,
      records,
      error
  ));

  // Check newly generated schedule matches the sample
  EXPECT_TRUE(
    compare_backups(
      backup_folder_path,
      get_sample_folder_path("write-schedule-record")
    )
  );
}
