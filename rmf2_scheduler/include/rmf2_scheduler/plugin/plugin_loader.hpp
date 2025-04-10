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
// tesseract_common/include/tesseract_common/plugin_loader.h

#ifndef RMF2_SCHEDULER__PLUGIN__PLUGIN_LOADER_HPP_
#define RMF2_SCHEDULER__PLUGIN__PLUGIN_LOADER_HPP_

#include <set>
#include <string>
#include <memory>
#include <vector>

namespace rmf2_scheduler
{

namespace plugin
{
/// This is a utility class for loading plugins within Tesseract
/**
 * The library_name should not include the prefix 'lib' or suffix '.so'.
 * It will add the correct prefix and suffix based on the OS.
 *
 * It supports providing additional search paths and set environment variable
 * which should be used when searching for plugins.
 *
 * The plugin must be exported using the macro RMF2_ADD_PLUGIN.
 * In the example below, the first parameter is the derived object and
 * the second is the assigned symbol name which is used for loading Example:
 *
 *   RMF2_ADD_PLUGIN(my_namespace::MyPlugin, plugin)
 *
 *   PluginLoader loader;
 *   loader.search_libraries.insert("my_plugin"); // libmy_plugin.so
 *   std::shared_ptr<PluginBase> p = loader.instantiate<PluginBase>("plugin");
 */
class PluginLoader
{
public:
  /// Indicate is system folders may be search if plugin is not found in any of the paths
  bool search_system_folders{true};

  /// A list of paths to search for plugins
  std::set<std::string> search_paths;

  /// A list of library names without the prefix or suffix that contain plugins
  std::set<std::string> search_libraries;

  /// The environment variable containing plugin search paths
  std::string search_paths_env;

  /// The environment variable containing plugins
  /**
   * The plugins are store ins the following format.
   * The library name does not contain prefix or suffix
   *   Format: library_name:library_name1:library_name2
   */
  std::string search_libraries_env;

  /// Instantiate a plugin with the provided name
  /**
   * \param plugin_name The plugin name to find
   * \return A instantiate of the plugin, if nullptr it failed to create plugin
   */
  template<class PluginBase>
  std::shared_ptr<PluginBase> instantiate(const std::string & plugin_name) const;

  /// Check if plugin is available
  /**
   * \param plugin_name The plugin name to find
   * \return True if plugin is found
   */
  inline bool is_plugin_available(const std::string & plugin_name) const;

  /// Get the available plugins for the provided PluginBase type
  /**
   * This expects the Plugin base to have a static std::string SECTION_NAME
   * which is used for looking up plugins
   *
   * \return A list of available plugins for the provided PluginBase type
   */
  template<class PluginBase>
  std::vector<std::string> get_available_plugins() const;

  /// Get the available plugins under the provided section
  /**
   * \param section The section name to get all available plugins
   * \return A list of available plugins under the provided section
   */
  inline std::vector<std::string> get_available_plugins(const std::string & section) const;

  /// Get the available sections within the provided search libraries
  /**
   * \return A list of available sections
   */
  inline std::vector<std::string> get_available_sections(bool include_hidden = false) const;

  /// The number of plugins stored. The size of plugins variable
  /**
   * \return The number of plugins.
   */
  inline size_t count() const;

  /// Utility function to add library containing symbol to the search env variable
  /**
   * In some cases the name and location of a library is unknown at runtime, but a symbol can
   * be linked at compile time. This is true for Python auditwheel distributions. This
   * utility function will determine the location of the library, and add it to the library search
   * environment variable so it can be found.
   *
   * \param symbol_ptr Pointer to the symbol to find
   * \param search_libraries_env The environmental variable to modify
   */
  static inline void add_symbol_library_to_search_libraries_env(
    const void * symbol_ptr,
    const std::string & search_libraries_env
  );
};

}  // namespace plugin
}  // namespace rmf2_scheduler

#include "rmf2_scheduler/plugin/plugin_loader_impl.hpp"

#endif  // RMF2_SCHEDULER__PLUGIN__PLUGIN_LOADER_HPP_
