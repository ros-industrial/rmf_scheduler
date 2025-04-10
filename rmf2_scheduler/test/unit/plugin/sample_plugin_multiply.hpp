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

#ifndef UNIT__PLUGIN__SAMPLE_PLUGIN_MULTIPLY_HPP_
#define UNIT__PLUGIN__SAMPLE_PLUGIN_MULTIPLY_HPP_

#include <string>

#include "sample_plugin_base.hpp"

namespace rmf2_scheduler
{

namespace plugin
{
class SamplePluginMultiply : public SamplePluginBase
{
public:
  SamplePluginMultiply() = default;
  ~SamplePluginMultiply() override = default;
  SamplePluginMultiply(const SamplePluginMultiply &) = default;
  SamplePluginMultiply & operator=(const SamplePluginMultiply &) = default;
  SamplePluginMultiply(SamplePluginMultiply &&) = default;
  SamplePluginMultiply & operator=(SamplePluginMultiply &&) = default;
  double multiply(double x, double y) override;
};

}  // namespace plugin
}  // namespace rmf2_scheduler

#endif  // UNIT__PLUGIN__SAMPLE_PLUGIN_MULTIPLY_HPP_
