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

#include <wordexp.h>

#include <cstdio>
#include <fstream>
#include <filesystem>

#include "rmf2_scheduler/cache/action.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"
#include "rmf2_scheduler/data/uuid.hpp"
#include "rmf2_scheduler/storage/schedule_stream_simple.hpp"

#include "rmf2_scheduler/log.hpp"

namespace fs = std::filesystem;

namespace rmf2_scheduler
{

namespace storage
{

namespace simple
{

static constexpr char MANIFEST_FILENAME[] = "manifest.r2ts";
static constexpr char BACKUP_FILENAME_PREFIX[] = "backup-";
static constexpr char BACKUP_FILENAME_EXTENSION[] = ".r2tsc";

ScheduleStream::ScheduleStream(
  size_t keep_last,
  const std::string & backup_folder_path
)
: keep_last_(keep_last)
{
  // push temporary ids
  for (size_t i = 0; i < keep_last + 1; i++) {
    backup_ids_.push_front("");
  }

  // Expand the backup path
  wordexp_t p;
  wordexp(backup_folder_path.c_str(), &p, 0);
  backup_folder_path_ = p.we_wordv[p.we_offs];
  wordfree(&p);
  fs::path dir(backup_folder_path_);

  if (!fs::is_directory(dir) || !fs::exists(dir)) {
    LOG_WARN(
      "Schedule backup directory %s doesn't exist. "
      "A new folder will be created.", dir.c_str()
    );
    fs::create_directories(dir);
    return;
  }

  fs::path manifest_name(MANIFEST_FILENAME);
  fs::path manifest_filepath = dir / manifest_name;
  if (!fs::exists(manifest_filepath)) {
    LOG_WARN(
      "Schedule backup manifest not found in %s. "
      "A new one will be created.", dir.c_str());
    return;
  }

  LOG_INFO("Schedule backup manifest found in %s.", dir.c_str());

  std::ifstream rf(manifest_filepath, std::ios::ate | std::ios::binary);
  size_t size = rf.tellg();
  size_t old_keep_last = size / 36;
  rf.seekg(0, std::ios::beg);
  char uuid_c_str[36];
  bool found_valid_cache = false;
  cache::ScheduleCache::Ptr temp_schedule = cache::ScheduleCache::make_shared();
  for (size_t i = 0; i < old_keep_last; i++) {
    rf.read(uuid_c_str, 36);
    std::string uuid(uuid_c_str, 36);
    if (found_valid_cache) {
      // Remove the rest of the backup
      // if there is already one valid
      _remove_backup(uuid);
      continue;
    }

    // Try loading the backup to validate
    std::string error;
    bool result = _load_backup(uuid, temp_schedule, error);
    if (result) {
      found_valid_cache = true;
      LOG_INFO("Found valid schedule backup at [%s].", uuid.c_str());
      backup_ids_.pop_back();
      backup_ids_.push_front(uuid);
    } else {
      LOG_ERROR("Schedule backup [%s] is invalid. %s", uuid.c_str(), error.c_str());
      if (!result) {
        // Remove if invalid
        _remove_backup(uuid);
      }
    }
  }

  if (!found_valid_cache) {
    LOG_ERROR("No valid backup found");
    return;
  }

  rf.close();
}

ScheduleStream::~ScheduleStream()
{
}

bool ScheduleStream::read_schedule(
  cache::ScheduleCache::Ptr schedule_cache,
  const data::TimeWindow & time_window,
  std::string & error
)
{
  if (!schedule_cache) {
    error = "ScheduleCache is a nullptr.";
    return false;
  }

  auto backup_schedule = cache::ScheduleCache::make_shared();

  bool result = _load_backup(
    backup_ids_.front(), backup_schedule, error
  );

  if (!result) {
    return false;
  }

  // Retrieve schedule within a time window
  auto tasks = backup_schedule->lookup_tasks(
    time_window.start,
    time_window.end
  );

  // Add tasks to cache
  for (const auto & task : tasks) {
    auto task_action = cache::Action::create(
      data::action_type::TASK_ADD,
      cache::ActionPayload().task(task)
    );

    if (task_action->validate(schedule_cache, error)) {
      task_action->apply();
    } else {
      return false;
    }
  }

  // Retrieve relevant process to read
  std::unordered_set<std::string> query_process_ids;
  query_process_ids.reserve(tasks.size());
  for (const auto & task : tasks) {
    if (task->process_id.has_value()) {
      query_process_ids.emplace(*task->process_id);
    }
  }

  std::vector<data::Process::ConstPtr> processes;
  processes.reserve(query_process_ids.size());
  for (auto & process_id : query_process_ids) {
    processes.push_back(backup_schedule->get_process(process_id));
  }

  std::unordered_set<std::string> query_extra_event_ids;
  // Check if there are additional events that should be retrieved
  for (const auto & process : processes) {
    process->graph.for_each_node(
      [&query_extra_event_ids, schedule_cache](const data::Node::Ptr & node) {
        if (!schedule_cache->has_event(node->id())) {
          query_extra_event_ids.emplace(node->id());
        }
      }
    );
  }

  std::vector<data::Task::ConstPtr> extra_tasks;
  extra_tasks.reserve(query_extra_event_ids.size());
  for (auto & event_id : query_extra_event_ids) {
    extra_tasks.push_back(backup_schedule->get_task(event_id));
  }

  // Add extra tasks to cache
  for (const auto & task : extra_tasks) {
    auto task_action = cache::Action::create(
      data::action_type::TASK_ADD,
      cache::ActionPayload().task(task)
    );

    if (task_action->validate(schedule_cache, error)) {
      task_action->apply();
    } else {
      return false;
    }
  }

  // Add process to cache
  for (const auto & process : processes) {
    auto process_action = cache::Action::create(
      data::action_type::PROCESS_ADD,
      cache::ActionPayload().process(process)
    );

    if (process_action->validate(schedule_cache, error)) {
      process_action->apply();
    } else {
      return false;
    }
  }

  return true;
}

bool ScheduleStream::write_schedule(
  cache::ScheduleCache::ConstPtr schedule_cache,
  const data::TimeWindow & time_window,
  std::string & error
)
{
  if (!schedule_cache) {
    error = "ScheduleCache is a nullptr.";
    return false;
  }

  // Load backup schedule
  auto backup_schedule = cache::ScheduleCache::make_shared();

  bool result = _load_backup(
    backup_ids_.front(), backup_schedule, error
  );

  if (!result) {
    return false;
  }


  // Lookup the events
  auto tasks = schedule_cache->lookup_tasks(time_window.start, time_window.end);

  // Find the relevant processes
  std::unordered_set<std::string> process_ids;
  for (const auto & task : tasks) {
    if (task->process_id.has_value()) {
      process_ids.insert(*task->process_id);
    }
  }
  std::vector<data::Process::ConstPtr> processes;
  processes.reserve(process_ids.size());
  for (const auto & process_id : process_ids) {
    processes.push_back(schedule_cache->get_process(process_id));
  }

  std::vector<cache::Action::Ptr> cache_actions;

  // Prepare task actions
  for (auto & task : tasks) {
    if (backup_schedule->has_task(task->id)) {
      cache_actions.push_back(
        cache::Action::create(
          data::action_type::TASK_UPDATE,
          cache::ActionPayload().task(task)
      ));
    } else {
      cache_actions.push_back(
        cache::Action::create(
          data::action_type::TASK_ADD,
          cache::ActionPayload().task(task)
      ));
    }
  }

  // Prepare process actions
  for (auto & process : processes) {
    if (backup_schedule->has_process(process->id)) {
      cache_actions.push_back(
        cache::Action::create(
          data::action_type::PROCESS_UPDATE,
          cache::ActionPayload().process(process)
      ));
    } else {
      cache_actions.push_back(
        cache::Action::create(
          data::action_type::PROCESS_ADD,
          cache::ActionPayload().process(process)
      ));
    }
  }

  // TODO(Briancbn): series

  // Apply the Cache actions
  for (auto & action : cache_actions) {
    if (!action->validate(backup_schedule, error)) {
      return false;
    }
    action->apply();
  }

  _write_schedule_to_backup(backup_schedule);
  return true;
}

bool ScheduleStream::write_schedule(
  cache::ScheduleCache::ConstPtr cache,
  const std::vector<data::ScheduleChangeRecord> & change_actions,
  std::string & error
)
{
  auto backup_schedule = cache::ScheduleCache::make_shared();

  bool result = _load_backup(
    backup_ids_.front(), backup_schedule, error
  );

  if (!result) {
    return false;
  }

  // Squash the changes
  auto squashed_action = data::ScheduleChangeRecord::squash(change_actions);

  std::vector<cache::Action::Ptr> cache_actions;

  // Prepare task actions
  for (auto & change_action : squashed_action.get(data::record_data_type::TASK)) {
    if (change_action.action == data::record_action_type::ADD) {
      cache_actions.push_back(
        cache::Action::create(
          data::action_type::TASK_ADD,
          cache::ActionPayload().task(
            cache->get_task(change_action.id)
          )
      ));
    } else if (change_action.action == data::record_action_type::UPDATE) {
      cache_actions.push_back(
        cache::Action::create(
          data::action_type::TASK_UPDATE,
          cache::ActionPayload().task(
            cache->get_task(change_action.id)
          )
      ));
    } else if (change_action.action == data::record_action_type::DELETE) {
      cache_actions.push_back(
        cache::Action::create(
          data::action_type::TASK_DELETE,
          cache::ActionPayload().id(change_action.id)
      ));
    } else {
      assert(false);
    }
  }

  // Prepare process actions
  for (auto & change_action : squashed_action.get(data::record_data_type::PROCESS)) {
    if (change_action.action == data::record_action_type::ADD) {
      cache_actions.push_back(
        cache::Action::create(
          data::action_type::PROCESS_ADD,
          cache::ActionPayload().process(
            cache->get_process(change_action.id)
          )
      ));
    } else if (change_action.action == data::record_action_type::UPDATE) {
      cache_actions.push_back(
        cache::Action::create(
          data::action_type::PROCESS_UPDATE,
          cache::ActionPayload().process(
            cache->get_process(change_action.id)
          )
      ));
    } else if (change_action.action == data::record_action_type::DELETE) {
      cache_actions.push_back(
        cache::Action::create(
          data::action_type::PROCESS_DELETE,
          cache::ActionPayload().id(change_action.id)
      ));
    } else {
      assert(false);
    }
  }

  // TODO(Briancbn): series

  // Apply the Cache actions
  for (auto & action : cache_actions) {
    if (!action->validate(backup_schedule, error)) {
      return false;
    }
    action->apply();
  }

  _write_schedule_to_backup(backup_schedule);
  return true;
}

bool ScheduleStream::refresh_tasks(
  cache::ScheduleCache::Ptr /*schedule_cache*/,
  const std::vector<std::string> & /*task_ids*/,
  std::string & /*error*/
)
{
  return true;
}

void ScheduleStream::_remove_backup(
  const std::string & uuid
)
{
  fs::path cache_file_name(
    std::string("backup-") + uuid + ".r2tsc"
  );
  fs::path full_cache_path = backup_folder_path_ / cache_file_name;
  std::remove(full_cache_path.c_str());
}

bool ScheduleStream::_load_backup(
  const std::string & uuid,
  cache::ScheduleCache::Ptr schedule,
  std::string & error
)
{
  if (uuid.empty()) {
    // Do nothing
    return true;
  }

  fs::path backup_dir(backup_folder_path_);
  fs::path backup_filename(
    std::string(BACKUP_FILENAME_PREFIX) + uuid + BACKUP_FILENAME_EXTENSION);
  fs::path backup_filepath = backup_dir / backup_filename;
  if (!fs::exists(backup_filepath)) {
    error = "Backup file ";
    error += backup_filename.c_str();
    error += " in Manifest not found in ";
    error += backup_dir.c_str();
    error += ".";
  }

  std::ifstream rcf(backup_filepath, std::ios::ate | std::ios::binary);
  size_t cf_size = rcf.tellg();
  rcf.seekg(0, std::ios::beg);
  std::vector<uint8_t> v_bson(cf_size);
  rcf.read(reinterpret_cast<char *>(v_bson.data()), cf_size);

  std::vector<data::Task::Ptr> tasks;
  std::vector<data::Process::Ptr> processes;

  nlohmann::json j_from_bson;
  try {
    j_from_bson = nlohmann::json::from_bson(v_bson);
  } catch (const std::exception & e) {
    rcf.close();
    error = "Backup file ";
    error += backup_filepath.c_str();
    error += " loading failure\n";
    error += e.what();
    return false;
  }

  rcf.close();

  LOG_DEBUG("Cache file:\n%s", j_from_bson.dump(2).c_str());
  if (j_from_bson.contains("tasks")) {
    try {
      j_from_bson.at("tasks").get_to(tasks);
    } catch (const std::exception & e) {
      error = "Backup file ";
      error += backup_filepath.c_str();
      error += " TASK loading failure\n";
      error += e.what();
      return false;
    }
  }

  if (j_from_bson.contains("processes")) {
    try {
      j_from_bson.at("processes").get_to(processes);
    } catch (const std::exception & e) {
      error = "Backup file ";
      error += backup_filepath.c_str();
      error += " PROCESS loading failure\n";
      error += e.what();
      return false;
    }
  }

  std::vector<cache::Action::Ptr> cache_actions;

  // Prepare task actions
  for (auto & task : tasks) {
    cache_actions.push_back(
      cache::Action::create(
        data::action_type::TASK_ADD,
        cache::ActionPayload().task(task)
    ));
  }
  // Prepare process actions
  for (auto & process : processes) {
    cache_actions.push_back(
      cache::Action::create(
        data::action_type::PROCESS_ADD,
        cache::ActionPayload().process(process)
    ));
  }

  // TODO(Briancbn): series

  // Apply the Cache actions
  for (auto & action : cache_actions) {
    if (!action->validate(schedule, error)) {
      return false;
    }
    action->apply();
  }

  return true;
}

void ScheduleStream::_write_schedule_to_backup(cache::ScheduleCache::ConstPtr schedule)
{
  // Serialize the tasks
  auto tasks = schedule->get_all_tasks();
  auto processes = schedule->get_all_processes();

  nlohmann::json j;
  j["tasks"] = nlohmann::json(tasks);
  j["processes"] = nlohmann::json(processes);
  std::vector<uint8_t> data = nlohmann::json::to_bson(j);

  std::string uuid_to_add = data::gen_uuid();
  std::string uuid_to_remove = backup_ids_.back();
  fs::path backup_dir(backup_folder_path_);

  // Remove old backup
  if (!uuid_to_remove.empty()) {
    _remove_backup(uuid_to_remove);
  }

  // Remove old backup id
  backup_ids_.pop_back();

  // Add new backup file
  fs::path new_backup_filename(
    std::string(BACKUP_FILENAME_PREFIX) + uuid_to_add + BACKUP_FILENAME_EXTENSION
  );
  fs::path backup_filepath = backup_dir / new_backup_filename;
  std::ofstream wcf(backup_filepath, std::ios::out | std::ios::binary);
  wcf.write(reinterpret_cast<char *>(data.data()), data.size());
  wcf.close();

  // Add new backup id
  backup_ids_.push_front(uuid_to_add);

  // Rewrite manifest file
  fs::path manifest_filename(MANIFEST_FILENAME);
  fs::path manifest_filepath = backup_dir / manifest_filename;
  std::ofstream wf(manifest_filepath, std::ios::out | std::ios::binary);
  for (auto itr = backup_ids_.begin(); itr != backup_ids_.end(); itr++) {
    wf.write(itr->data(), itr->size());
  }
  wf.close();
}

}  // namespace simple
}  // namespace storage
}  // namespace rmf2_scheduler
