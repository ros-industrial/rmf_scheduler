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

#include "rmf_scheduler/task/builder.hpp"
#include "rmf_scheduler/task/task_plugin_impl.hpp"

namespace rmf_scheduler
{

namespace task
{

Builder::Builder()
{
}

Builder::~Builder()
{
}

void Builder::load_plugin(
  const std::shared_ptr<void> & node,
  const std::string & name,
  const std::string & plugin,
  const std::vector<std::string> & supported_tasks)
{
  load_task_plugin_impl<BuilderInterface>(
    this, node, name, plugin, "rmf_scheduler::task::BuilderInterface", supported_tasks);
}

void Builder::unload_plugin(
  const std::string & name)
{
  TaskPluginManager::unload_plugin(name);
}

void Builder::build_task(data::Event & event)
{
  // Check if the event type is supported
  if (!is_supported(event.type)) {
    RS_LOG_DEBUG("event type not supported: %s", event.type.c_str());
    return;
  }

  // Parse event details and pass it to the plugin
  nlohmann::json event_details_json = nlohmann::json::parse(event.event_details);
  nlohmann::json task_details_json;
  try {
    task_details_json = nlohmann::json::parse(event.task_details);
  } catch (const std::exception & e) {
    RS_LOG_WARN(
      "Generating initial task detail for event %s[%s]",
      event.id.c_str(), event.type.c_str());
  }

  auto builder_interface = get_supported_plugin<BuilderInterface>(event.type).second;

  builder_interface->build_task(event_details_json, task_details_json);

  event.task_details = task_details_json.dump();
}

}  // namespace task

}  // namespace rmf_scheduler
