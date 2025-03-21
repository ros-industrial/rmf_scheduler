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

#include "rmf2_scheduler/data/json_serializer.hpp"
#include "rmf2_scheduler/cache/schedule_cache.hpp"
#include "rmf2_scheduler/cache/event_action.hpp"
#include "rmf2_scheduler/cache/task_action.hpp"
#include "rmf2_scheduler/cache/process_action.hpp"
#include "rmf2_scheduler/storage/schedule_stream_ld_broker.hpp"
#include "rmf2_scheduler/utils/string_utils.hpp"
#include "rmf2_scheduler/utils/url_utils.hpp"
#include "rmf2_scheduler/http/request.hpp"

#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler
{

namespace storage
{

namespace ld_broker
{

namespace
{

static const size_t PAGINATION_LIMIT = 1000;

bool validate_http_response(
  const std::unique_ptr<http::Response> & response,
  const std::shared_ptr<http::Request> & request,
  std::string & error
)
{
  if (!response) {
    return false;
  }

  if (!response->is_successful()) {
    if (request) {
      error += "Invalid response from " + request->get_request_method() + " " +
        request->get_request_url() + "\n";
    }

    error += std::to_string(response->get_status_code()) + " " +
      response->get_status_text() + "\n" +
      response->extract_data_as_string();
    return false;
  }

  return true;
}
}  // namespace

ScheduleStream::ScheduleStream(
  const std::string & ld_broker_url,
  std::shared_ptr<http::Transport> transport
)
: create_url_(url::combine_multiple(ld_broker_url, {"v1", "entityOperations", "create"})),
  read_url_(url::combine_multiple(ld_broker_url, {"v1", "entities"})),
  upsert_url_(url::combine_multiple(ld_broker_url, {"v1", "entityOperations", "upsert"})),
  update_url_(url::combine_multiple(ld_broker_url, {"v1", "entityOperations", "update"})),
  delete_url_(url::combine_multiple(ld_broker_url, {"v1", "entities"})),
  transport_(transport)
{
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
  // Retrieve Events
  size_t offset = 0;
  const size_t limit = PAGINATION_LIMIT;
  std::unordered_set<std::string> query_process_ids;

  // Manual pagination
  do {
    auto event_query_request = std::make_shared<http::Request>(
      url::append_query_params(
        read_url_,
          {
            {"type", "urn:rmf2:event"},
            {"limit", std::to_string(limit)},
            {"offset", std::to_string(offset)},
            {
              "q",
              "startTime>=" + std::string(time_window.start.to_ISOtime()) + ";" +
              "startTime<" + std::string(time_window.end.to_ISOtime())
            }
          }
      ),
      http::request_type::kGet,
      transport_
    );

    auto event_query_response = event_query_request->get_response_and_block(error);
    if (!validate_http_response(event_query_response, event_query_request, error)) {
      return false;
    }

    // Convert response to JSON and actions
    std::vector<data::Task::Ptr> tasks;
    if (
      !_read_response_to_tasks(
        event_query_response->extract_data_as_string(),
        tasks,
        error
    ))
    {
      return false;
    }

    // Add tasks to cache
    for (const auto & task : tasks) {
      auto task_action = std::make_shared<cache::TaskAction>(
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
    query_process_ids.reserve(tasks.size());
    for (const auto & task : tasks) {
      if (task->process_id.has_value()) {
        query_process_ids.emplace("urn:process:" + *task->process_id);
      }
    }

    if (tasks.empty()) {
      break;
    }

    offset += limit;
  } while(true);

  if (query_process_ids.empty()) {
    return true;
  }

  // Retrieve Process
  offset = 0;
  std::vector<data::Process::Ptr> processes_to_add;
  std::unordered_set<std::string> query_extra_event_ids;

  // Manual pagination
  do {
    auto process_query_request = std::make_shared<http::Request>(
      url::append_query_params(
        read_url_,
          {
            {"type", "urn:rmf2:process"},
            {"limit", std::to_string(limit)},
            {"offset", std::to_string(offset)},
            {"id", string_utils::join(",", query_process_ids)}
          }
      ),
      http::request_type::kGet,
      transport_
    );

    auto process_query_response = process_query_request->get_response_and_block(error);
    if (!validate_http_response(process_query_response, process_query_request, error)) {
      return false;
    }

    // Convert response to JSON and process actions
    std::vector<data::Process::Ptr> processes;
    if (!_read_response_to_processes(
        process_query_response->extract_data_as_string(), processes, error))
    {
      return false;
    }

    // Check if there are additional events that should be retrieved
    for (const auto & process : processes) {
      process->graph.for_each_node(
        [&query_extra_event_ids, schedule_cache](const data::Node::Ptr & node) {
          if (!schedule_cache->has_event(node->id())) {
            query_extra_event_ids.emplace("urn:event:" + node->id());
          }
        }
      );
    }

    if (processes.empty()) {
      break;
    }

    processes_to_add.reserve(processes_to_add.size() + processes.size());
    processes_to_add.insert(processes_to_add.end(), processes.begin(), processes.end());

    offset += limit;
  } while (true);

  if (!query_extra_event_ids.empty()) {
    // Retrieve extra Events
    // Manual pagination
    offset = 0;
    do {
      auto extra_event_query_request = std::make_shared<http::Request>(
        url::append_query_params(
          read_url_,
            {
              {"type", "urn:rmf2:event"},
              {"limit", std::to_string(limit)},
              {"offset", std::to_string(offset)},
              {"id", string_utils::join(";", query_extra_event_ids)}
            }
        ),
        http::request_type::kGet,
        transport_
      );

      auto extra_event_query_response = extra_event_query_request->get_response_and_block(error);
      if (!validate_http_response(extra_event_query_response, extra_event_query_request, error)) {
        return false;
      }

      // Convert response to JSON and actions
      std::vector<data::Task::Ptr> extra_tasks;
      if (
        !_read_response_to_tasks(
          extra_event_query_response->extract_data_as_string(),
          extra_tasks,
          error
      ))
      {
        return false;
      }

      // Add extra tasks to cache
      for (const auto & task : extra_tasks) {
        auto task_action = std::make_shared<cache::TaskAction>(
          data::action_type::TASK_ADD,
          cache::ActionPayload().task(task)
        );

        if (task_action->validate(schedule_cache, error)) {
          task_action->apply();
        } else {
          return false;
        }
      }

      if (extra_tasks.empty()) {
        break;
      }

      offset += limit;
    }while (true);
  }

  // Add process to cache
  for (const auto & process : processes_to_add) {
    auto process_action = std::make_shared<cache::ProcessAction>(
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


bool ScheduleStream::refresh_tasks(
  cache::ScheduleCache::Ptr schedule_cache,
  const std::vector<std::string> & task_ids,
  std::string & error
)
{
  // Retrieve Events
  auto event_query_request = std::make_shared<http::Request>(
    url::append_query_params(
      read_url_,
        {
          {"type", "urn:rmf2:event"},
          {"limit", "1000"},
          {"id", string_utils::join(",", task_ids)},
        }
    ),
    http::request_type::kGet,
    transport_
  );

  auto event_query_response = event_query_request->get_response_and_block(error);
  if (!validate_http_response(event_query_response, event_query_request, error)) {
    return false;
  }

  // Convert response to JSON and actions
  std::vector<data::Task::Ptr> tasks;
  if (
    !_read_response_to_tasks(
      event_query_response->extract_data_as_string(),
      tasks,
      error
  ))
  {
    return false;
  }

  // Add tasks to cache
  for (const auto & task : tasks) {
    auto task_action = std::make_shared<cache::TaskAction>(
      data::action_type::TASK_ADD,
      cache::ActionPayload().task(task)
    );

    if (task_action->validate(schedule_cache, error)) {
      task_action->apply();
    } else {
      return false;
    }
  }

  return true;
}

namespace
{

nlohmann::json to_context_broker_entity(
  const std::string & type,
  const std::string & uuid,
  const nlohmann::json & payload_value
)
{
  // Payload
  nlohmann::json payload;
  payload["type"] = "Property";
  payload["value"] = payload_value;

  // Body
  nlohmann::json json_body;
  json_body["id"] = "urn:" + type + ":" + uuid;
  json_body["type"] = "urn:rmf2:" + type;
  json_body["payload"] = payload;
  json_body["@context"] = "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld";

  return json_body;
}

nlohmann::json to_context_broker_entity(
  const data::Task::ConstPtr & task
)
{
  // Create Start time lookup
  nlohmann::json start_time;
  start_time["@type"] = "Datetime";
  start_time["@value"] = nlohmann::json(task->start_time);
  nlohmann::json start_time_lookup;
  start_time_lookup["type"] = "Property";
  start_time_lookup["value"] = start_time;

  // Body
  nlohmann::json json_body = to_context_broker_entity(
    "event",
    task->id,
    nlohmann::json(*task)
  );
  json_body["startTime"] = start_time_lookup;

  return json_body;
}

nlohmann::json to_context_broker_entity(
  const data::Process::ConstPtr & process
)
{
  // Body
  nlohmann::json json_body = to_context_broker_entity(
    "process",
    process->id,
    nlohmann::json(*process)
  );

  return json_body;
}

}  // namespace

bool ScheduleStream::write_schedule(
  cache::ScheduleCache::ConstPtr schedule_cache,
  const data::TimeWindow & time_window,
  std::string & error
)
{
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

  // Upsert tasks to the LD broker
  if (!tasks.empty()) {
    std::vector<nlohmann::json> task_v;
    task_v.reserve(tasks.size());
    for (const auto & task : tasks) {
      task_v.push_back(to_context_broker_entity(task));
    }

    auto upsert_task_request = std::make_shared<rmf2_scheduler::http::Request>(
      upsert_url_,
      rmf2_scheduler::http::request_type::kPost,
      transport_
    );
    upsert_task_request->set_content_type("application/ld+json");
    upsert_task_request->add_request_body(
      nlohmann::json(task_v),
      error
    );

    auto upsert_task_response = upsert_task_request->get_response_and_block(error);

    if (!validate_http_response(upsert_task_response, upsert_task_request, error)) {
      return false;
    }
    // TODO(Briancbn): handle retry
  }

  // Upsert processes to the LD broker
  if (!processes.empty()) {
    std::vector<nlohmann::json> process_v;
    process_v.reserve(process_v.size());
    for (const auto & process : processes) {
      process_v.push_back(to_context_broker_entity(process));
    }

    auto upsert_process_request = std::make_shared<rmf2_scheduler::http::Request>(
      upsert_url_,
      rmf2_scheduler::http::request_type::kPost,
      transport_
    );
    upsert_process_request->set_content_type("application/ld+json");
    upsert_process_request->add_request_body(
      nlohmann::json(process_v),
      error
    );

    auto upsert_process_response = upsert_process_request->get_response_and_block(error);

    if (!validate_http_response(upsert_process_response, upsert_process_request, error)) {
      return false;
    }
    // TODO(Briancbn): handle retry
  }
  return true;
}

bool ScheduleStream::write_schedule(
  cache::ScheduleCache::ConstPtr cache,
  const std::vector<data::ScheduleChangeRecord> & change_actions,
  std::string & error
)
{
  // Squash the changes
  auto squashed_action = data::ScheduleChangeRecord::squash(change_actions);

  // Prepare events
  std::vector<data::Task::ConstPtr> tasks_to_upsert;
  std::vector<std::string> events_to_delete;
  for (auto & change_action : squashed_action.get(data::record_data_type::EVENT)) {
    if (change_action.action == data::record_action_type::DELETE) {
      events_to_delete.push_back(change_action.id);
      continue;
    }

    std::string upsert_id = change_action.id;

    // Check if ID is a task
    auto task = cache->get_task(upsert_id);
    tasks_to_upsert.push_back(task);
  }

  // Prepare processes
  std::vector<data::Process::ConstPtr> processes_to_upsert;
  std::vector<std::string> processes_to_delete;
  for (auto & change_action : squashed_action.get(data::record_data_type::PROCESS)) {
    if (change_action.action == data::record_action_type::DELETE) {
      processes_to_delete.push_back(change_action.id);
      continue;
    }

    processes_to_upsert.push_back(cache->get_process(change_action.id));
  }

  // Upsert tasks to the LD broker
  if (!tasks_to_upsert.empty()) {
    std::vector<nlohmann::json> task_v;
    task_v.reserve(tasks_to_upsert.size());
    for (const auto & task : tasks_to_upsert) {
      task_v.push_back(to_context_broker_entity(task));
    }

    auto upsert_task_request = std::make_shared<rmf2_scheduler::http::Request>(
      upsert_url_,
      rmf2_scheduler::http::request_type::kPost,
      transport_
    );
    upsert_task_request->set_content_type("application/ld+json");
    upsert_task_request->add_request_body(nlohmann::json(task_v), error);

    auto upsert_task_response = upsert_task_request->get_response_and_block(error);

    if (!validate_http_response(upsert_task_response, upsert_task_request, error)) {
      return false;
    }
    // TODO(Briancbn): handle retry
  }

  // Delete events from the broker
  for (auto & event_id : events_to_delete) {
    auto delete_event_request = std::make_shared<rmf2_scheduler::http::Request>(
      url::combine(delete_url_, "urn:event:" + event_id),
      rmf2_scheduler::http::request_type::kDelete,
      transport_
    );
    delete_event_request->set_content_type("application/ld+json");

    auto delete_event_response = delete_event_request->get_response_and_block(error);

    if (!validate_http_response(delete_event_response, delete_event_request, error)) {
      return false;
    }
    // TODO(Briancbn): handle retry
  }

  // Upsert processes to the LD broker
  if (!processes_to_upsert.empty()) {
    std::vector<nlohmann::json> process_v;
    process_v.reserve(process_v.size());
    for (const auto & process : processes_to_upsert) {
      process_v.push_back(to_context_broker_entity(process));
    }

    auto upsert_process_request = std::make_shared<rmf2_scheduler::http::Request>(
      upsert_url_,
      rmf2_scheduler::http::request_type::kPost,
      transport_
    );
    upsert_process_request->set_content_type("application/ld+json");
    upsert_process_request->add_request_body(nlohmann::json(process_v), error);

    auto upsert_process_response = upsert_process_request->get_response_and_block(error);

    if (!validate_http_response(upsert_process_response, upsert_process_request, error)) {
      return false;
    }
    // TODO(Briancbn): handle retry
  }

  // Delete processes from the broker
  for (auto & process_id : processes_to_delete) {
    auto delete_process_request = std::make_shared<rmf2_scheduler::http::Request>(
      url::combine(delete_url_, "urn:process:" + process_id),
      rmf2_scheduler::http::request_type::kDelete,
      transport_
    );
    delete_process_request->set_content_type("application/ld+json");

    auto delete_process_response = delete_process_request->get_response_and_block(error);

    if (!validate_http_response(delete_process_response, delete_process_request, error)) {
      return false;
    }
    // TODO(Briancbn): handle retry
  }

  return true;
}

bool ScheduleStream::_read_response_to_tasks(
  const std::string & response_stream,
  std::vector<data::Task::Ptr> & tasks,
  std::string & error
) const
{
  try {
    nlohmann::json j = nlohmann::json::parse(response_stream);
    const auto event_cb_v = j.template get<std::vector<nlohmann::json>>();
    tasks.reserve(event_cb_v.size());
    for (const auto & event_cb_j : event_cb_v) {
      data::Task::Ptr task = std::make_shared<data::Task>();
      event_cb_j.at("payload").at("value").get_to(*task);
      tasks.push_back(task);
    }
  } catch (const std::exception & e) {
    error = std::string(e.what()) + ":\n" + response_stream;
    return false;
  }

  return true;
}

bool ScheduleStream::_read_response_to_processes(
  const std::string & response_stream,
  std::vector<data::Process::Ptr> & processes,
  std::string & error
) const
{
  try {
    nlohmann::json j = nlohmann::json::parse(response_stream);
    const auto process_cb_v = j.template get<std::vector<nlohmann::json>>();
    processes.reserve(process_cb_v.size());
    for (const auto & process_cb_j : process_cb_v) {
      data::Process::Ptr process = std::make_shared<data::Process>();
      process_cb_j.at("payload").at("value").get_to(*process);
      processes.push_back(process);
    }
  } catch (const std::exception & e) {
    error = e.what();
    return false;
  }

  return true;
}

}  // namespace ld_broker
}  // namespace storage
}  // namespace rmf2_scheduler
