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
// tesseract_common/include/tesseract_common/class_loader.h

#ifndef RMF2_SCHEDULER__PLUGIN__CLASS_LOADER_HPP_
#define RMF2_SCHEDULER__PLUGIN__CLASS_LOADER_HPP_

#include <string>
#include <memory>
#include <vector>

namespace rmf2_scheduler
{

namespace plugin
{

/// This is a wrapper around Boost DLL for loading plugins within RMF2
/**
 * The library_name should not include the prefix 'lib' or suffix '.so'.
 * It will add the correct prefix and suffix based on the OS.
 *
 * The plugin must be exported using the macro RMF2_ADD_PLUGIN.
 * In the example below, the first parameter is the derived object and the second is
 * the assigned symbol name which is
 * used for looding Example: RMF2_ADD_PLUGIN(my_namespace::MyPlugin, plugin)
 *
 *   auto p = ClassLoader::createSharedInstance<my_namespace::MyPluginBase>("my_plugin", "plugin");
 *
 */
struct ClassLoader
{
  /// Create a shared instance for the provided symbol_name loaded from the library_name
  /// searching system folders for library
  /**
   * The symbol name is the alias provide when calling RMF2_ADD_PLUGIN
   *
   * \param symbol_name The symbol to create a shared instance of
   * \param library_name The library name to load which does not include
   *                     the prefix 'lib' or suffix '.so'
   * \param library_directory The library directory,
   *                          if empty it will enable search system directories
   * \return A shared pointer of the object with the symbol name located in library_name_
   */
  template<class ClassBase>
  static std::shared_ptr<ClassBase> create_shared_instance(
    const std::string & symbol_name,
    const std::string & library_name,
    const std::string & library_directory = ""
  );

  /// Check if the symbol is available in the library_name searching system folders for library
  /**
   * The symbol name is the alias provide when calling RMF2_ADD_PLUGIN
   *
   * \param symbol_name The symbol to create a shared instance of
   * \param library_name The library name to load which does not include i
   *                     the prefix 'lib' or suffix '.so'
   * \param library_directory The library directory,
   *                          if empty it will enable search system directories
   * \return True if the symbol exists, otherwise false
   */
  static inline bool is_class_available(
    const std::string & symbol_name,
    const std::string & library_name,
    const std::string & library_directory = ""
  );

  /// Get a list of available symbols under the provided section
  /**
   * \param section The section to search for available symbols
   * \param library_name The library name to load which does not include
   *                     the prefix 'lib' or suffix '.so'
   * \param library_directory The library directory,
   *                          if empty it will enable search system directories
   * \return A list of symbols if they exist.
   */
  static inline std::vector<std::string> get_available_symbols(
    const std::string & section,
    const std::string & library_name,
    const std::string & library_directory = ""
  );

  /// Get a list of available sections
  /**
   * \param library_name The library name to load which does not include
   *                     the prefix 'lib' or suffix '.so'
   * \param library_directory The library directory,
   *                          if empty it will enable search system directories
   * \return A list of sections if they exist.
   */
  static inline std::vector<std::string> get_available_sections(
    const std::string & library_name,
    const std::string & library_directory = "",
    bool include_hidden = false
  );

  /// Give library name without prefix and suffix it will return the library name with
  /// the prefix and suffix
  /**
   * * For instance, for a library_name like "boost" it returns :
   * - path/to/libboost.so on posix platforms
   * - path/to/libboost.dylib on OSX
   * - path/to/boost.dll on Windows
   *
   * \todo When support is dropped for 18.04 switch to using boost::dll::shared_library::decorate
   * \param library_name The library name without prefix and suffix
   * \param library_directory The library directory,
   *                          if empty it just returns the decorated library name
   * \return The library name or path with prefix and suffix
   */
  static inline std::string decorate(
    const std::string & library_name,
    const std::string & library_directory = "");
};

}  // namespace plugin
}  // namespace rmf2_scheduler

// clang-format off
#define RMF2_ADD_PLUGIN_SECTIONED(DERIVED_CLASS, ALIAS, SECTION) \
  extern "C" BOOST_SYMBOL_EXPORT DERIVED_CLASS ALIAS; \
  BOOST_DLL_SECTION(SECTION, read) BOOST_DLL_SELECTANY \
  DERIVED_CLASS ALIAS;

#define RMF2_ADD_PLUGIN(DERIVED_CLASS, ALIAS) \
  RMF2_ADD_PLUGIN_SECTIONED(DERIVED_CLASS, ALIAS, boostdll)

#define RMF2_PLUGIN_ANCHOR_DECL(ANCHOR_NAME) \
  const void * ANCHOR_NAME();

#define RMF2_PLUGIN_ANCHOR_IMPL(ANCHOR_NAME) \
  const void * ANCHOR_NAME() {return (const void *)(&ANCHOR_NAME);}

#include "rmf2_scheduler/plugin/class_loader_impl.hpp"

#endif  // RMF2_SCHEDULER__PLUGIN__CLASS_LOADER_HPP_
