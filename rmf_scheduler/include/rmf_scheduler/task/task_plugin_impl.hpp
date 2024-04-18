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

#ifndef RMF_SCHEDULER__TASK__TASK_PLUGIN_IMPL_HPP_
#define RMF_SCHEDULER__TASK__TASK_PLUGIN_IMPL_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"

#include "rmf_scheduler/task/task_plugin.hpp"
#include "rmf_scheduler/log.hpp"

namespace rmf_scheduler
{

template<typename BasePluginT>
std::shared_ptr<BasePluginT>
load_task_plugin_impl(
  TaskPluginManager * task_manager,
  const std::shared_ptr<void> & node,
  const std::string & name,
  const std::string & plugin,
  const std::string & base_plugin_name,
  const std::vector<std::string> & supported_task_types)
{
  RS_LOG_INFO(
    "Loading %s plugin:\n"
    "%s [%s] ...",
    base_plugin_name.c_str(),
    name.c_str(),
    plugin.c_str());

  if (name.empty()) {
    throw exception::PluginException("New runtime interface name cannot be empty.");
  }

  if (task_manager->plugins_.find(name) != task_manager->plugins_.end()) {
    throw exception::PluginException("New runtime interface %s already exists.", name.c_str());
  }

  using PluginLoaderT = pluginlib::ClassLoader<BasePluginT>;
  std::shared_ptr<PluginLoaderT> plugin_loader;
  if (!task_manager->loader_) {
    plugin_loader = std::make_shared<PluginLoaderT>(
      "rmf_scheduler", base_plugin_name);
  } else {
    plugin_loader = std::static_pointer_cast<PluginLoaderT>(task_manager->loader_);
  }

  auto plugin_instance = plugin_loader->createSharedInstance(plugin);

  for (auto & supported_task : supported_task_types) {
    // Check if an existing interface can execute the same supported task
    auto itr = task_manager->task_plugins_lookup_.find(supported_task);
    if (itr != task_manager->task_plugins_lookup_.end()) {
      throw exception::PluginException(
              "New plugins %s [%s] shares the same task type [%s]"
              "with %s, please consider removing %s before adding %s.",
              name.c_str(), plugin.c_str(),
              itr->first.c_str(),
              itr->second.c_str(),
              itr->second.c_str(), name.c_str());
    }
  }

  plugin_instance->init(node);

  task_manager->plugins_[name] = plugin_instance;

  // Add lookup for the runtime interface for each task
  std::ostringstream oss;
  for (auto & supported_task : supported_task_types) {
    task_manager->task_plugins_lookup_[supported_task] = name;
    oss << "  - " << supported_task << '\n';
  }

  RS_LOG_INFO(
    "Successfully loaded %s plugin: %s.\n"
    "Support task types:\n%s",
    base_plugin_name.c_str(),
    name.c_str(),
    oss.str().c_str());
  task_manager->loader_ = plugin_loader;

  return plugin_instance;
}

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__TASK__TASK_PLUGIN_IMPL_HPP_
