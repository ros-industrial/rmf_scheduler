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
// tesseract_common/test/test_plugin_base.h

#ifndef UNIT__PLUGIN__SAMPLE_PLUGIN_BASE_HPP_
#define UNIT__PLUGIN__SAMPLE_PLUGIN_BASE_HPP_

#include <string>

namespace rmf2_scheduler
{

namespace plugin
{

class SamplePluginBase
{
public:
  SamplePluginBase() = default;
  virtual ~SamplePluginBase() = default;
  SamplePluginBase(const SamplePluginBase &) = default;
  SamplePluginBase & operator=(const SamplePluginBase &) = default;
  SamplePluginBase(SamplePluginBase &&) = default;
  SamplePluginBase & operator=(SamplePluginBase &&) = default;
  virtual double multiply(double x, double y) = 0;

protected:
  static const char SECTION_NAME[];
  friend class PluginLoader;
};

}  // namespace plugin
}  // namespace rmf2_scheduler

#endif  // UNIT__PLUGIN__SAMPLE_PLUGIN_BASE_HPP_
