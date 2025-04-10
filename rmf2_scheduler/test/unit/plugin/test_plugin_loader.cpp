// Copyright 2024 ROS Industrial Consortium Asia Pacific
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

#include "gtest/gtest.h"

#include "rmf2_scheduler/plugin/plugin_loader.hpp"
#include "sample_plugin_base.hpp"

const char rmf2_scheduler::plugin::SamplePluginBase::SECTION_NAME[] = "TestBase";

class TestPluginPluginLoader : public ::testing::Test
{
};

TEST_F(TestPluginPluginLoader, parse_environment_variable_list) {
  using namespace rmf2_scheduler::plugin;  // NOLINT(build/namespaces)
#ifndef _WIN32
  std::string env_var = "UNITTESTENV=a:b:c";
#else
  std::string env_var = "UNITTESTENV=a;b;c";
#endif
  putenv(env_var.data());
  std::set<std::string> s = detail::parse_environment_variable_list("UNITTESTENV");
  std::vector<std::string> v(s.begin(), s.end());
  EXPECT_EQ(v[0], "a");
  EXPECT_EQ(v[1], "b");
  EXPECT_EQ(v[2], "c");
}

TEST_F(TestPluginPluginLoader, load_sample_plugin)  // NOLINT
{
  using namespace rmf2_scheduler::plugin;  // NOLINT(build/namespaces)

  {
    PluginLoader plugin_loader;
    plugin_loader.search_paths.insert(std::string(TEST_PLUGIN_DIR));
    plugin_loader.search_libraries.insert("sample_plugin_multiply");

    EXPECT_TRUE(plugin_loader.is_plugin_available("plugin"));
    auto plugin = plugin_loader.instantiate<SamplePluginBase>("plugin");
    ASSERT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);

    std::vector<std::string> sections = plugin_loader.get_available_sections();
    ASSERT_EQ(sections.size(), 1);
    EXPECT_EQ(sections.at(0), "TestBase");

    sections = plugin_loader.get_available_sections(true);
    ASSERT_GT(sections.size(), 1);

    std::vector<std::string> symbols = plugin_loader.get_available_plugins<SamplePluginBase>();
    ASSERT_EQ(symbols.size(), 1);
    EXPECT_EQ(symbols.at(0), "plugin");

    symbols = plugin_loader.get_available_plugins("TestBase");
    ASSERT_EQ(symbols.size(), 1);
    EXPECT_EQ(symbols.at(0), "plugin");
  }

// For some reason on Ubuntu 18.04 it does not search the current directory
// when only the library name is provided
#if BOOST_VERSION > 106800 && !__APPLE__
  {
    PluginLoader plugin_loader;
    plugin_loader.search_paths.insert(".");
    plugin_loader.search_libraries.insert("sample_plugin_multiply");

    EXPECT_TRUE(plugin_loader.is_plugin_available("plugin"));
    auto plugin = plugin_loader.instantiate<SamplePluginBase>("plugin");
    ASSERT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  }
#endif

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    plugin_loader.search_paths.insert("does_not_exist");
    plugin_loader.search_libraries.insert("sample_plugin_multiply");

    {
      EXPECT_FALSE(plugin_loader.is_plugin_available("plugin"));
      auto plugin = plugin_loader.instantiate<SamplePluginBase>("plugin");
      EXPECT_TRUE(plugin == nullptr);
    }
  }

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    plugin_loader.search_libraries.insert("sample_plugin_multiply");

    {
      EXPECT_FALSE(plugin_loader.is_plugin_available("does_not_exist"));
      auto plugin = plugin_loader.instantiate<SamplePluginBase>("does_not_exist");
      EXPECT_TRUE(plugin == nullptr);
    }

    plugin_loader.search_system_folders = true;

    {
      EXPECT_FALSE(plugin_loader.is_plugin_available("does_not_exist"));
      auto plugin = plugin_loader.instantiate<SamplePluginBase>("does_not_exist");
      EXPECT_TRUE(plugin == nullptr);
    }
  }

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    plugin_loader.search_libraries.insert("does_not_exist");

    {
      EXPECT_FALSE(plugin_loader.is_plugin_available("plugin"));
      auto plugin = plugin_loader.instantiate<SamplePluginBase>("plugin");
      EXPECT_TRUE(plugin == nullptr);
    }

    plugin_loader.search_system_folders = true;

    {
      EXPECT_FALSE(plugin_loader.is_plugin_available("plugin"));
      auto plugin = plugin_loader.instantiate<SamplePluginBase>("plugin");
      EXPECT_TRUE(plugin == nullptr);
    }
  }

  {
    PluginLoader plugin_loader;
    EXPECT_FALSE(plugin_loader.is_plugin_available("plugin"));
    auto plugin = plugin_loader.instantiate<SamplePluginBase>("plugin");
    EXPECT_TRUE(plugin == nullptr);
  }

  {
    PluginLoader plugin_loader;
    plugin_loader.search_system_folders = false;
    EXPECT_FALSE(plugin_loader.is_plugin_available("plugin"));
    auto plugin = plugin_loader.instantiate<SamplePluginBase>("plugin");
    EXPECT_TRUE(plugin == nullptr);
  }
}
