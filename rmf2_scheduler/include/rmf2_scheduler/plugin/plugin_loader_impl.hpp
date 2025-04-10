// Copyright 2024 ROS Industrial Consortium Asia Pacific
// Copyright 2021 Southwest Research Institute
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
//
// Original file taken from:
// https://github.com/tesseract-robotics/tesseract/blob/master/
// tesseract_common/include/tesseract_common/plugin_loader.hpp

#ifndef RMF2_SCHEDULER__PLUGIN__PLUGIN_LOADER_IMPL_HPP_
#define RMF2_SCHEDULER__PLUGIN__PLUGIN_LOADER_IMPL_HPP_

#include <set>
#include <string>
#include <memory>
#include <vector>

#include <boost/dll/runtime_symbol_info.hpp>
#include <boost/algorithm/string.hpp>

#include "rmf2_scheduler/log.hpp"
#include "rmf2_scheduler/plugin/class_loader.hpp"
#include "rmf2_scheduler/plugin/plugin_loader.hpp"

namespace rmf2_scheduler
{

namespace plugin
{

namespace detail
{

inline std::set<std::string> parse_environment_variable_list(const std::string & env_variable)
{
  std::set<std::string> list;
  char * env_var = std::getenv(env_variable.c_str());
  if (env_var == nullptr) {  // Environment variable not found
    return list;
  }

  std::string evn_str = std::string(env_var);
#ifndef _WIN32
  boost::split(list, evn_str, boost::is_any_of(":"), boost::token_compress_on);
#else
  boost::split(list, evn_str, boost::is_any_of(";"), boost::token_compress_on);
#endif
  return list;
}

inline std::set<std::string> get_all_search_paths(
  const std::string & search_paths_env,
  const std::set<std::string> & existing_search_paths
)
{
  // Check for environment variable to override default library
  if (!search_paths_env.empty()) {
    std::set<std::string> search_paths = parse_environment_variable_list(search_paths_env);
    search_paths.insert(existing_search_paths.begin(), existing_search_paths.end());
    return search_paths;
  }

  return existing_search_paths;
}

inline std::set<std::string> get_all_search_libraries(
  const std::string & search_libraries_env,
  const std::set<std::string> & existing_search_libraries
)
{
  // Check for environment variable to override default library
  if (!search_libraries_env.empty()) {
    std::set<std::string> search_libraries = parse_environment_variable_list(search_libraries_env);
    search_libraries.insert(existing_search_libraries.begin(), existing_search_libraries.end());
    return search_libraries;
  }

  return existing_search_libraries;
}

/**
 * @brief This will remove libraries with full path in the provided library_names and return them.
 * @param library_names The set to search and remove libraries with full paths
 * @return A set of the libraries provided as full path
 */
inline std::set<std::string> extract_libraries_with_full_path(
  std::set<std::string> & library_names
)
{
  std::set<std::string> libraries_with_fullpath;
  for (auto it = library_names.begin(); it != library_names.end(); ) {
    if (boost::dll::fs::exists(*it) && boost::dll::fs::path(*it).is_absolute()) {
      libraries_with_fullpath.insert(*it);
      it = library_names.erase(it);
    } else {
      ++it;
    }
  }

  return libraries_with_fullpath;
}

}  // namespace detail

template<class PluginBase>
std::shared_ptr<PluginBase> PluginLoader::instantiate(const std::string & plugin_name) const
{
  // Check for environment variable for plugin definitions
  std::set<std::string> library_names = detail::get_all_search_libraries(
    search_libraries_env,
    search_libraries
  );
  if (library_names.empty()) {
    LOG_ERROR("No plugin libraries were provided!");
    return nullptr;
  }

  std::set<std::string> libraries_with_fullpath = detail::extract_libraries_with_full_path(
    library_names
  );
  for (const auto & library_fullpath : libraries_with_fullpath) {
    if (ClassLoader::is_class_available(plugin_name, library_fullpath)) {
      return ClassLoader::create_shared_instance<PluginBase>(plugin_name, library_fullpath);
    }
  }

  // Check for environment variable for search paths
  std::set<std::string> search_paths_local = detail::get_all_search_paths(
    search_paths_env,
    search_paths
  );
  for (const auto & path : search_paths_local) {
    for (const auto & library : search_libraries) {
      if (ClassLoader::is_class_available(plugin_name, library, path)) {
        return ClassLoader::create_shared_instance<PluginBase>(plugin_name, library, path);
      }
    }
  }

  // If not found in any of the provided search paths then search system folders if allowed
  if (search_system_folders) {
    for (const auto & library : search_libraries) {
      if (ClassLoader::is_class_available(plugin_name, library)) {
        return ClassLoader::create_shared_instance<PluginBase>(plugin_name, library);
      }
    }
  }

  std::stringstream msg;
  if (search_system_folders) {
    msg << "\nSearch Paths (Search System Folders: True):\n";
  } else {
    msg << "\nSearch Paths (Search System Folders: False):\n";
  }

  for (const auto & path : search_paths_local) {
    msg << "    - " + path << "\n";
  }

  msg << "Search Libraries:\n";
  for (const auto & library : search_libraries) {
    msg << "    - " + ClassLoader::decorate(library) << "\n";
  }

  LOG_ERROR(
    "Failed to instantiate plugin '%s', Details: %s",
    plugin_name.c_str(),
    msg.str().c_str()
  );

  return nullptr;
}

bool PluginLoader::is_plugin_available(const std::string & plugin_name) const
{
  // Check for environment variable for plugin definitions
  std::set<std::string> library_names = detail::get_all_search_libraries(
    search_libraries_env,
    search_libraries
  );
  if (library_names.empty()) {
    LOG_ERROR("No plugin libraries were provided!");
    return false;
  }

  // Check for libraries provided as full paths. These are searched first
  std::set<std::string> libraries_with_fullpath = detail::extract_libraries_with_full_path(
    library_names
  );
  for (const auto & library_fullpath : libraries_with_fullpath) {
    if (ClassLoader::is_class_available(plugin_name, library_fullpath)) {
      return true;
    }
  }

  // Check for environment variable to override default library
  std::set<std::string> search_paths_local = detail::get_all_search_paths(
    search_paths_env,
    search_paths
  );
  for (const auto & path : search_paths_local) {
    for (const auto & library : search_libraries) {
      if (ClassLoader::is_class_available(plugin_name, library, path)) {
        return true;
      }
    }
  }

  // If not found in any of the provided search paths then search system folders if allowed
  if (search_system_folders) {
    for (const auto & library : search_libraries) {
      if (ClassLoader::is_class_available(plugin_name, library)) {
        return true;
      }
    }
  }

  return false;
}

template<class PluginBase>
std::vector<std::string> PluginLoader::get_available_plugins() const
{
  return get_available_plugins(PluginBase::SECTION_NAME);
}

std::vector<std::string> PluginLoader::get_available_plugins(
  const std::string & section
) const
{
  std::vector<std::string> plugins;

  // Check for environment variable for plugin definitions
  std::set<std::string> library_names = detail::get_all_search_libraries(
    search_libraries_env,
    search_libraries
  );
  if (library_names.empty()) {
    LOG_ERROR("No plugin libraries were provided!");
    return plugins;
  }

  // Check for libraries provided as full paths. These are searched first
  std::set<std::string> libraries_with_fullpath = detail::extract_libraries_with_full_path(
    library_names
  );
  for (const auto & library_fullpath : libraries_with_fullpath) {
    std::vector<std::string> lib_plugins = ClassLoader::get_available_symbols(
      section,
      library_fullpath
    );
    plugins.insert(plugins.end(), lib_plugins.begin(), lib_plugins.end());
  }

  // Check for environment variable to override default library
  std::set<std::string> search_paths_local = detail::get_all_search_paths(
    search_paths_env,
    search_paths
  );
  for (const auto & path : search_paths_local) {
    for (const auto & library : search_libraries) {
      std::vector<std::string> lib_plugins = ClassLoader::get_available_symbols(
        section, library, path
      );
      plugins.insert(plugins.end(), lib_plugins.begin(), lib_plugins.end());
    }
  }

  return plugins;
}

std::vector<std::string> PluginLoader::get_available_sections(bool include_hidden) const
{
  std::vector<std::string> sections;

  // Check for environment variable for plugin definitions
  std::set<std::string> library_names = detail::get_all_search_libraries(
    search_libraries_env, search_libraries
  );
  if (library_names.empty()) {
    LOG_ERROR("No plugin libraries were provided!");
    return sections;
  }

  // Check for libraries provided as full paths. These are searched first
  std::set<std::string> libraries_with_fullpath = detail::extract_libraries_with_full_path(
    library_names
  );
  for (const auto & library_fullpath : libraries_with_fullpath) {
    std::vector<std::string> lib_sections = ClassLoader::get_available_sections(
      library_fullpath, "", include_hidden
    );
    sections.insert(sections.end(), lib_sections.begin(), lib_sections.end());
  }

  // Check for environment variable to override default library
  std::set<std::string> search_paths_local = detail::get_all_search_paths(
    search_paths_env,
    search_paths
  );
  for (const auto & path : search_paths_local) {
    for (const auto & library : search_libraries) {
      std::vector<std::string> lib_sections = ClassLoader::get_available_sections(
        library, path, include_hidden
      );
      sections.insert(sections.end(), lib_sections.begin(), lib_sections.end());
    }
  }

  return sections;
}

size_t PluginLoader::count() const
{
  return detail::get_all_search_libraries(search_libraries_env, search_libraries).size();
}

void PluginLoader::add_symbol_library_to_search_libraries_env(
  const void * symbol_ptr,
  const std::string & search_libraries_env
)
{
  std::string env_var_str;
  char * env_var = std::getenv(search_libraries_env.c_str());
  if (env_var != nullptr) {
    env_var_str = env_var;
  }

  boost::dll::fs::path lib_path = boost::dll::fs::canonical(
    boost::dll::symbol_location_ptr(symbol_ptr)
  );

  if (env_var_str.empty()) {
    env_var_str = lib_path.string();
  } else {
#ifndef _WIN32
    env_var_str = env_var_str + ":" + lib_path.string();
#else
    env_var_str = env_var_str + ";" + lib_path.string();
#endif
  }

#ifndef _WIN32
  setenv(search_libraries_env.c_str(), env_var_str.c_str(), 1);
#else
  _putenv_s(search_libraries_env.c_str(), env_var_str.c_str());
#endif
}

}  // namespace plugin
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__PLUGIN__PLUGIN_LOADER_IMPL_HPP_
