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

#include <wordexp.h>

#include <cstdio>
#include <fstream>
#include <filesystem>
#include "rmf_scheduler/cache.hpp"
#include "rmf_scheduler/log.hpp"
#include "rmf_scheduler/parser.hpp"
#include "rmf_scheduler/utils/uuid.hpp"

#include "nlohmann/json.hpp"

namespace fs = std::filesystem;

namespace rmf_scheduler
{

Cache::Cache(
  size_t keep_last,
  std::string cache_dir)
{
  // push temporary ids
  for (size_t i = 0; i < keep_last; i++) {
    cache_ids_.push_front("");
  }

  // expand the cache path
  wordexp_t p;
  wordexp(cache_dir.c_str(), &p, 0);
  cache_dir_ = p.we_wordv[p.we_offs];
  wordfree(&p);
  fs::path dir(cache_dir_);
  if (!fs::is_directory(dir) || !fs::exists(dir)) {
    RS_LOG_WARN(
      "Cache directory %s doesn't exist. "
      "A new folder will be created.", dir.c_str());
    fs::create_directory(dir);
    return;
  }
  fs::path manifest_name("rs_cache_manifest");
  fs::path full_path = dir / manifest_name;
  if (!fs::exists(full_path)) {
    RS_LOG_WARN(
      "Cache Manifest not found in %s. "
      "A new one will be created.", dir.c_str());
    return;
  }

  RS_LOG_INFO("Cache Manifest found in %s.", dir.c_str());

  std::ifstream rf(full_path, std::ios::ate | std::ios::binary);
  size_t size = rf.tellg();
  size_t old_keep_last = size / 36;
  rf.seekg(0, std::ios::beg);
  char uuid[37];
  bool found_valid_cache = false;
  for (size_t i = 0; i < old_keep_last; i++) {
    rf.read(uuid, 36);
    // Null terminate the string
    uuid[36] = '\0';
    if (found_valid_cache) {
      // Remove the rest of the cache
      // if there is already one valid
      remove_cache(uuid);
      continue;
    }
    try {
      data::Schedule schedule;
      load_cache(uuid, schedule);
      found_valid_cache = true;
      RS_LOG_INFO("Found valid cache at %s", uuid);
      cache_ids_.pop_back();
      cache_ids_.push_front(uuid);
    } catch (const std::exception & e) {
      RS_LOG_ERROR("%s", e.what());
      // Remove if invalid too
      remove_cache(uuid);
    }
  }

  if (!found_valid_cache) {
    RS_LOG_ERROR(
      "No valid cache found");
    return;
  }

  rf.close();
}

void Cache::from_last_cache(data::Schedule & schedule)
{
  std::string uuid = cache_ids_.front();
  if (uuid.empty()) {
    // Do nothing
  } else {
    return load_cache(uuid, schedule);
  }
}

void Cache::write_local_cache(const data::Schedule & schedule)
{
  auto events_to_export = schedule.events_handler_const().get_all_events();
  const auto & dags_to_export = schedule.dags_const();
  const auto & series_map_to_export = schedule.series_map_const();
  RS_LOG_INFO(
    "Writing new cache to disk\n"
    "- Events total: %lu\n"
    "- DAG total: %lu\n"
    "- Series total: %lu\n",
    events_to_export.size(),
    dags_to_export.size(),
    series_map_to_export.size());
  data::Schedule::Description schedule_description;
  for (auto & itr : events_to_export) {
    schedule_description.events.emplace(itr.first, itr.second);
  }

  for (auto & itr : dags_to_export) {
    schedule_description.dependencies.emplace(itr.first, itr.second.description());
  }

  for (auto & itr : series_map_to_export) {
    schedule_description.series_map.emplace(itr.first, itr.second.description());
  }

  nlohmann::json j;
  parser::schedule_to_json(schedule_description, j, true);
  std::vector<uint8_t> v_cbor = nlohmann::json::to_cbor(j);

  std::string uuid_to_add = utils::gen_uuid();
  std::string uuid_to_remove = cache_ids_.back();
  fs::path cache_dir(cache_dir_);

  // Remove old cache
  if (!uuid_to_remove.empty()) {
    remove_cache(uuid_to_remove);
  }

  // Remove old cache id
  cache_ids_.pop_back();

  // Add new cache file
  fs::path new_cache_file_name(
    std::string("rs_cache-") + uuid_to_add);
  fs::path full_cache_path = cache_dir_ / new_cache_file_name;
  std::ofstream wcf(full_cache_path, std::ios::out | std::ios::binary);
  wcf.write(reinterpret_cast<char *>(v_cbor.data()), v_cbor.size());
  wcf.close();

  // Add new cache id
  cache_ids_.push_front(uuid_to_add);

  // Rewrite manifest file
  fs::path manifest_name("rs_cache_manifest");
  fs::path full_manifest_path = cache_dir_ / manifest_name;
  std::ofstream wf(full_manifest_path, std::ios::out | std::ios::binary);
  for (auto itr = cache_ids_.begin(); itr != cache_ids_.end(); itr++) {
    wf.write(itr->data(), itr->size());
  }
  wf.close();
}

void Cache::load_cache(
  const std::string & uuid,
  data::Schedule & schedule)
{
  fs::path cache_dir(cache_dir_);
  fs::path cache_file_name(
    std::string("rs_cache-") + uuid);
  fs::path full_cache_path = cache_dir / cache_file_name;
  if (!fs::exists(full_cache_path)) {
    throw rmf_scheduler::exception::CacheInvalidException(
            "Cache file %s in Manifest not found in %s.",
            cache_file_name.c_str(), cache_dir.c_str());
  }

  std::ifstream rcf(full_cache_path, std::ios::ate | std::ios::binary);
  size_t cf_size = rcf.tellg();
  rcf.seekg(0, std::ios::beg);
  std::vector<uint8_t> v_cbor(cf_size);
  rcf.read(reinterpret_cast<char *>(v_cbor.data()), cf_size);

  std::unordered_map<std::string, data::Event> events_to_add;
  std::unordered_map<std::string, data::DAG> dags_to_add;
  std::unordered_map<std::string, data::Series> event_series_map_to_add, dag_series_map_to_add;

  try {
    nlohmann::json j_from_cbor;
    j_from_cbor = nlohmann::json::from_cbor(v_cbor);

    RS_LOG_DEBUG("Cache file:\n%s", j_from_cbor.dump(2).c_str());

    data::Schedule::Description schedule_description;
    parser::json_to_schedule(j_from_cbor, schedule_description);

    // Validate the events
    schedule.validate_add_events(schedule_description.events, events_to_add);

    // Validate the dependencies
    schedule.validate_add_dags(
      schedule_description.dependencies,
      dags_to_add,
      events_to_add);

    // Validate the series
    schedule.validate_add_series_map(
      schedule_description.series_map,
      event_series_map_to_add,
      dag_series_map_to_add,
      events_to_add,
      dags_to_add);
  } catch (const std::exception & e) {
    rcf.close();
    throw rmf_scheduler::exception::CacheInvalidException(
            "Cache file %s loading failure\n%s",
            full_cache_path.c_str(),
            e.what());
  }
  rcf.close();

  // Add events
  for (auto & itr : events_to_add) {
    schedule.add_event(itr.second);
  }

  // Add DAGs
  for (auto & itr : dags_to_add) {
    schedule.add_dag(itr.first, std::move(itr.second));

    // Generate the start time for events based on DAG
    schedule.generate_dag_event_start_time(itr.first);
  }

  // Add Series
  // Link the events / DAGs together using series.
  for (auto & series_itr : event_series_map_to_add) {
    schedule.add_event_series(series_itr.first, std::move(series_itr.second));
  }

  for (auto & series_itr : dag_series_map_to_add) {
    schedule.add_dag_series(series_itr.first, std::move(series_itr.second));
  }
}

void Cache::remove_cache(const std::string & uuid)
{
  fs::path cache_file_name(
    std::string("rs_cache-") + uuid);
  fs::path full_cache_path = cache_dir_ / cache_file_name;
  std::remove(full_cache_path.c_str());
}

}  // namespace rmf_scheduler
