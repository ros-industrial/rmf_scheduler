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

class TestPluginClassLoader : public ::testing::Test
{
};

TEST_F(TestPluginClassLoader, load_sample_plugin)  // NOLINT
{
  using namespace rmf2_scheduler::plugin;  // NOLINT(build/namespaces)
  const std::string lib_name = "sample_plugin_multiply";
  const std::string lib_dir = std::string(TEST_PLUGIN_DIR);
  const std::string symbol_name = "plugin";

  {
    std::vector<std::string> sections = ClassLoader::get_available_sections(lib_name, lib_dir);
    ASSERT_EQ(sections.size(), 1);
    EXPECT_EQ(sections.at(0), "TestBase");

    sections = ClassLoader::get_available_sections(lib_name, lib_dir, true);
    EXPECT_GT(sections.size(), 1);
  }

  {
    std::vector<std::string> symbols = ClassLoader::get_available_symbols(
      "TestBase", lib_name, lib_dir
    );
    ASSERT_EQ(symbols.size(), 1);
    EXPECT_EQ(symbols.at(0), symbol_name);
  }

  {
    EXPECT_TRUE(ClassLoader::is_class_available(symbol_name, lib_name, lib_dir));
    auto plugin = ClassLoader::create_shared_instance<SamplePluginBase>(
      symbol_name, lib_name, lib_dir
    );
    ASSERT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  }

// For some reason on Ubuntu 18.04 it does not search the current directory
// when only the library name is provided
#if BOOST_VERSION > 106800 && !__APPLE__
  {
    EXPECT_TRUE(ClassLoader::is_class_available(symbol_name, lib_name, "."));
    auto plugin = ClassLoader::create_shared_instance<SamplePluginBase>(
      symbol_name, lib_name, "."
    );
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  }
#endif

  {
    EXPECT_FALSE(ClassLoader::is_class_available(symbol_name, lib_name, "does_not_exist"));
    EXPECT_FALSE(ClassLoader::is_class_available(symbol_name, "does_not_exist", lib_dir));
    EXPECT_FALSE(ClassLoader::is_class_available("does_not_exist", lib_name, lib_dir));
  }

  {
    EXPECT_FALSE(ClassLoader::is_class_available(symbol_name, "does_not_exist"));
    EXPECT_FALSE(ClassLoader::is_class_available("does_not_exist", lib_name));
  }

  {
    EXPECT_ANY_THROW(
      ClassLoader::create_shared_instance<SamplePluginBase>(
        symbol_name, lib_name, "does_not_exist"
    ));
    EXPECT_ANY_THROW(
      ClassLoader::create_shared_instance<SamplePluginBase>(
        symbol_name, "does_not_exist", lib_dir
    ));
    EXPECT_ANY_THROW(
      ClassLoader::create_shared_instance<SamplePluginBase>(
        "does_not_exist", lib_name, lib_dir
    ));
  }

  {
    EXPECT_ANY_THROW(
      ClassLoader::create_shared_instance<SamplePluginBase>(
        symbol_name, "does_not_exist"
    ));
    EXPECT_ANY_THROW(
      ClassLoader::create_shared_instance<SamplePluginBase>(
        "does_not_exist", lib_name
    ));
  }
}
