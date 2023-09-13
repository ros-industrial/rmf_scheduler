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

#include "rmf_scheduler/task/task_plugin.hpp"

namespace rmf_scheduler
{

TaskPluginManager::~TaskPluginManager()
{
}


void TaskPluginManager::unload_plugin(const std::string & name)
{
  plugins_.erase(name);
  // Erase lookup entry too
  for (auto entry = task_plugins_lookup_.begin();
    entry != task_plugins_lookup_.end(); entry++)
  {
    if (entry->second == name) {
      task_plugins_lookup_.erase(entry);
    }
  }
}

bool TaskPluginManager::is_supported(
  const std::string & task_type) const
{
  return task_plugins_lookup_.find(task_type) != task_plugins_lookup_.end();
}

}  // namespace rmf_scheduler
