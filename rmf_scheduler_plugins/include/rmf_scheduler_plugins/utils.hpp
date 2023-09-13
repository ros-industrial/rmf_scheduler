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

#ifndef RMF_SCHEDULER_PLUGINS__UTILS_HPP_
#define RMF_SCHEDULER_PLUGINS__UTILS_HPP_

#include <string>

namespace rmf_scheduler_plugins
{

inline std::string to_slug(const std::string & in)
{
  std::string out;
  for (auto itr = in.begin(); itr != in.end(); itr++) {
    if (*itr == ' ' || *itr == '-') {
      out += '_';
      continue;
    }
    out += tolower(*itr);
  }
  return out;
}

}  // namespace rmf_scheduler_plugins

#endif  // RMF_SCHEDULER_PLUGINS__UTILS_HPP_
