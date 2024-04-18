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

#include "rmf_scheduler/task/estimate_interface.hpp"
#include "rmf_scheduler/task/task_plugin_impl.hpp"
#include "nlohmann/json.hpp"

namespace rmf_scheduler
{

namespace task
{

nlohmann::json EstimateState::json() const
{
  nlohmann::json result {
    {"waypoint", waypoint},
    {"orientation", orientation}
  };
  for (auto & itr : consumables) {
    result["consumables"][itr.first] = itr.second;
  }
  return result;
}

std::shared_ptr<EstimateState> EstimateState::from_json(
  const nlohmann::json & json)
{
  auto state = std::make_shared<EstimateState>();
  state->waypoint = json["waypoint"].get<int>();
  state->orientation = json["orientation"].get<double>();
  if (json.contains("consumables")) {
    for (auto & itr : json["consumables"].items()) {
      state->consumables.emplace(
        itr.key(),
        itr.value().get<double>());
    }
  }
  return state;
}

}  // namespace task

}  // namespace rmf_scheduler
