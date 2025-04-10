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
// tesseract_common/include/tesseract_common/class_loader.hpp

#ifndef RMF2_SCHEDULER__PLUGIN__CLASS_LOADER_IMPL_HPP_
#define RMF2_SCHEDULER__PLUGIN__CLASS_LOADER_IMPL_HPP_

#include <set>
#include <string>
#include <memory>
#include <vector>

#include <boost/dll/import.hpp>
#include <boost/dll/alias.hpp>
#include <boost/dll/import_class.hpp>
#include <boost/dll/shared_library.hpp>

#include "rmf2_scheduler/log.hpp"
#include "rmf2_scheduler/plugin/class_loader.hpp"

namespace rmf2_scheduler
{

namespace plugin
{

template<class ClassBase>
std::shared_ptr<ClassBase> ClassLoader::create_shared_instance(
  const std::string & symbol_name,
  const std::string & library_name,
  const std::string & library_directory
)
{
  boost::dll::fs::error_code ec;
  boost::dll::shared_library lib;
  if (library_directory.empty()) {
    boost::dll::fs::path sl(library_name);
    boost::dll::load_mode::type mode =
      boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders;
    lib = boost::dll::shared_library(sl, ec, mode);
  } else {
    boost::dll::fs::path sl = boost::dll::fs::path(library_directory) / library_name;
    lib = boost::dll::shared_library(sl, ec, boost::dll::load_mode::append_decorations);
  }

  // Check if it failed to find or load library
  if (ec) {
    throw std::runtime_error(
            "Failed to find or load library: " + decorate(library_name, library_directory) +
            " with error: " + ec.message());
  }

  // Check if library has symbol
  if (!lib.has(symbol_name)) {
    throw std::runtime_error(
            "Failed to find symbol '" + symbol_name +
            "' in library: " + decorate(library_name, library_directory));
  }

#if BOOST_VERSION >= 107600
  boost::shared_ptr<ClassBase> plugin = boost::dll::import_symbol<ClassBase>(lib, symbol_name);
#else
  boost::shared_ptr<ClassBase> plugin = boost::dll::import<ClassBase>(lib, symbol_name);
#endif
  return std::shared_ptr<ClassBase>(
    plugin.get(),
    [plugin](ClassBase *) mutable {   // NOLINT(readability/casting)
      plugin.reset();
    }
  );
}

bool ClassLoader::is_class_available(
  const std::string & symbol_name,
  const std::string & library_name,
  const std::string & library_directory
)
{
  boost::dll::fs::error_code ec;
  boost::dll::shared_library lib;
  if (library_directory.empty()) {
    boost::dll::fs::path sl(library_name);
    boost::dll::load_mode::type mode =
      boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders;
    lib = boost::dll::shared_library(sl, ec, mode);
  } else {
    boost::dll::fs::path sl = boost::dll::fs::path(library_directory) / library_name;
    lib = boost::dll::shared_library(sl, ec, boost::dll::load_mode::append_decorations);
  }

  // Check if it failed to find or load library
  if (ec) {
    LOG_DEBUG(
      "Failed to find or load library: %s with error: %s",
      decorate(library_name, library_directory).c_str(),
      ec.message().c_str()
    );
    return false;
  }

  return lib.has(symbol_name);
}

std::vector<std::string> ClassLoader::get_available_symbols(
  const std::string & section,
  const std::string & library_name,
  const std::string & library_directory
)
{
  boost::dll::fs::error_code ec;
  boost::dll::shared_library lib;
  if (library_directory.empty()) {
    boost::dll::fs::path sl(library_name);
    boost::dll::load_mode::type mode =
      boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders;
    lib = boost::dll::shared_library(sl, ec, mode);
  } else {
    boost::dll::fs::path sl = boost::dll::fs::path(library_directory) / library_name;
    lib = boost::dll::shared_library(sl, ec, boost::dll::load_mode::append_decorations);
  }

  // Check if it failed to find or load library
  if (ec) {
    LOG_DEBUG(
      "Failed to find or load library: %s with error: %s",
      decorate(library_name, library_directory).c_str(),
      ec.message().c_str()
    );
    return {};
  }

  // Class `library_info` can extract information from a library
  boost::dll::library_info inf(lib.location());

  // Getting symbols exported from he provided section
  return inf.symbols(section);
}

std::vector<std::string> ClassLoader::get_available_sections(
  const std::string & library_name,
  const std::string & library_directory,
  bool include_hidden
)
{
  boost::dll::fs::error_code ec;
  boost::dll::shared_library lib;
  if (library_directory.empty()) {
    boost::dll::fs::path sl(library_name);
    boost::dll::load_mode::type mode =
      boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders;
    lib = boost::dll::shared_library(sl, ec, mode);
  } else {
    boost::dll::fs::path sl = boost::dll::fs::path(library_directory) / library_name;
    lib = boost::dll::shared_library(sl, ec, boost::dll::load_mode::append_decorations);
  }

  // Check if it failed to find or load library
  if (ec) {
    LOG_DEBUG(
      "Failed to find or load library: %s with error: %s",
      decorate(library_name, library_directory).c_str(),
      ec.message().c_str()
    );
    return {};
  }

  // Class `library_info` can extract information from a library
  boost::dll::library_info inf(lib.location());

  // Getting section from library
  std::vector<std::string> sections = inf.sections();

  auto search_fn = [include_hidden](const std::string & section) {
      if (section.empty()) {
        return true;
      }

      if (include_hidden) {
        return false;
      }

      return (section.substr(0, 1) == ".") || (section.substr(0, 1) == "_");
    };

  sections.erase(std::remove_if(sections.begin(), sections.end(), search_fn), sections.end());
  return sections;
}

std::string ClassLoader::decorate(
  const std::string & library_name,
  const std::string & library_directory)
{
  boost::dll::fs::path sl;
  if (library_directory.empty()) {
    sl = boost::dll::fs::path(library_name);
  } else {
    sl = boost::dll::fs::path(library_directory) / library_name;
  }

  boost::dll::fs::path actual_path =
    (std::strncmp(sl.filename().string().c_str(), "lib", 3) != 0 ?
    boost::dll::fs::path(
      (sl.has_parent_path() ? sl.parent_path() / L"lib" : L"lib").native() +
      sl.filename().native()) :
    sl);

  actual_path += boost::dll::shared_library::suffix();
  return actual_path.string();
}

}  // namespace plugin
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__PLUGIN__CLASS_LOADER_IMPL_HPP_
