// Copyright 2023-2024 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__DATA__EDGE_HPP_
#define RMF2_SCHEDULER__DATA__EDGE_HPP_

#include <string>

namespace rmf2_scheduler
{
namespace data
{

/// information about the Edge
struct Edge
{
  /// Empty Constructor
  Edge()
  : Edge("hard")
  {
  }

  /// Full Constructor
  explicit Edge(const std::string & _type)
  : type(_type)
  {
  }

  /// Edge type
  /**
   * There are 2 types of edges
   * - hard: tasks has to be executed immediately after.
   * - soft: tasks has to executed after, but can be at a later time
   *
   * Default is set to hard
   */
  std::string type;

  inline bool operator==(const Edge & rhs) const
  {
    if (this->type != rhs.type) {return false;}
    return true;
  }

  inline bool operator!=(const Edge & rhs) const
  {
    return !(*this == rhs);
  }
};

}  // namespace data
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__DATA__EDGE_HPP_
