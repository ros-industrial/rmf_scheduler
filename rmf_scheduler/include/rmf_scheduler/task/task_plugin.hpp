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

#ifndef RMF_SCHEDULER__TASK__TASK_PLUGIN_HPP_
#define RMF_SCHEDULER__TASK__TASK_PLUGIN_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rmf_scheduler/exception.hpp"
#include "rmf_scheduler/error_code.hpp"

namespace rmf_scheduler
{

class TaskPluginBase
{
public:
  virtual ~TaskPluginBase() {}
  virtual void init(const std::shared_ptr<void> & node) = 0;
};

class TaskPluginManager
{
public:
  virtual ~TaskPluginManager();

  virtual void load_plugin(
    const std::shared_ptr<void> & node,
    const std::string & name,
    const std::string & plugin,
    const std::vector<std::string> & supported_task_types) = 0;

  virtual void unload_plugin(const std::string & name);

  template<typename BasePluginT>
  std::pair<std::string, std::shared_ptr<BasePluginT>>
  get_supported_plugin(
    const std::string & task_type);

  template<typename BasePluginT>
  std::shared_ptr<BasePluginT>
  get_plugin(const std::string & name);

  template<typename BasePluginT>
  void for_each_plugin(
    std::function<void(const std::string &, std::shared_ptr<BasePluginT>)> func);

  bool is_supported(const std::string & task_type) const;

private:
  std::unordered_map<std::string, std::shared_ptr<TaskPluginBase>> plugins_;
  std::unordered_map<std::string, std::string> task_plugins_lookup_;

  template<typename BasePluginT>
  friend std::shared_ptr<BasePluginT>
  load_task_plugin_impl(
    TaskPluginManager * task_manager,
    const std::shared_ptr<void> & node,
    const std::string & name,
    const std::string & plugin,
    const std::string & base_plugin_name,
    const std::vector<std::string> & supported_task_types);
};

namespace exception
{

class PluginException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  PluginException(const char * msg, Args && ... args)
  : ExceptionTemplate(
      ErrorCode::FAILURE | ErrorCode::RUNTIME | ErrorCode::NO_FIELD,
      msg, std::forward<Args>(args) ...)
  {
    add_prefix("PluginException:\n  ");
  }
};

class InvalidTaskSchemaException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  InvalidTaskSchemaException(const char * msg, Args && ... args)
  : ExceptionTemplate(
      ErrorCode::FAILURE | ErrorCode::INVALID_SCHEMA | ErrorCode::NO_FIELD,
      msg, std::forward<Args>(args) ...)
  {
    add_prefix("InvalidTaskSchemaException:\n ");
  }
};

}  // namespace exception

template<typename BasePluginT>
std::pair<std::string, std::shared_ptr<BasePluginT>>
TaskPluginManager::get_supported_plugin(
  const std::string & task_type)
{
  std::string plugin_name = task_plugins_lookup_.at(task_type);
  return std::make_pair(
    plugin_name,
    std::dynamic_pointer_cast<BasePluginT>(plugins_[plugin_name]));
}

template<typename BasePluginT>
std::shared_ptr<BasePluginT>
TaskPluginManager::get_plugin(
  const std::string & name)
{
  return
    std::dynamic_pointer_cast<BasePluginT>(plugins_[name]);
}

template<typename BasePluginT>
void TaskPluginManager::for_each_plugin(
  std::function<void(const std::string &, std::shared_ptr<BasePluginT>)> func)
{
  for (auto plugin : plugins_) {
    func(plugin.first, std::dynamic_pointer_cast<BasePluginT>(plugin.second));
  }
}

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__TASK__TASK_PLUGIN_HPP_
