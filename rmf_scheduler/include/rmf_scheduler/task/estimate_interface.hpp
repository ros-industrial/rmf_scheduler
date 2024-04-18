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

#ifndef RMF_SCHEDULER__TASK__ESTIMATE_INTERFACE_HPP_
#define RMF_SCHEDULER__TASK__ESTIMATE_INTERFACE_HPP_

#include <future>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include "nlohmann/json.hpp"
#include "rmf_scheduler/task/task_plugin.hpp"
#include "rmf_scheduler/exception.hpp"
#include "rmf_scheduler/error_code.hpp"

namespace rmf_scheduler
{

namespace task
{

struct EstimateState
{
  int waypoint;
  double orientation;
  std::unordered_map<std::string, double> consumables;
  nlohmann::json json() const;
  static std::shared_ptr<EstimateState> from_json(const nlohmann::json &);
};

struct EstimateStates
{
  /// Estimated start state
  std::shared_ptr<EstimateState> start;
  /// Estimated end state
  std::shared_ptr<EstimateState> end;
};

struct EstimateRequest
{
  uint64_t start_time;
  nlohmann::json details;
  EstimateState * state = nullptr;
};

struct EstimateResponse
{
  uint64_t deployment_time;
  uint64_t finish_time;
  uint64_t duration;
  EstimateState state;
};

class EstimateInterface : public TaskPluginBase
{
public:
  virtual ~EstimateInterface() {}

  virtual void init(const std::shared_ptr<void> & node) = 0;

  virtual std::shared_future<EstimateResponse> async_estimate(
    const std::string & id,
    const EstimateRequest & request) = 0;
};

}  // namespace task

namespace exception
{

class InvalidEstimateInterfaceException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  InvalidEstimateInterfaceException(const char * msg, Args && ... args)
  : ExceptionTemplate(
      ErrorCode::FAILURE | ErrorCode::RUNTIME | ErrorCode::NO_FIELD,
      msg, std::forward<Args>(args) ...)
  {
    add_prefix("InvalidEstimateInterfaceException:\n ");
  }
};

}  // namespace exception

}  // namespace rmf_scheduler

inline bool operator==(
  rmf_scheduler::task::EstimateState const & lhs,
  rmf_scheduler::task::EstimateState const & rhs)
{
  // Compare consumables
  if (lhs.consumables.size() != rhs.consumables.size()) {
    return false;
  }

  for (auto & lhs_itr : lhs.consumables) {
    auto rhs_itr = rhs.consumables.find(lhs_itr.first);
    if (rhs_itr == rhs.consumables.end()) {
      return false;
    }
    if (rhs_itr->second != lhs_itr.second) {
      return false;
    }
  }
  return lhs.waypoint == rhs.waypoint &&
         lhs.orientation == rhs.orientation;
}

inline bool operator!=(
  rmf_scheduler::task::EstimateState const & lhs,
  rmf_scheduler::task::EstimateState const & rhs)
{
  return !(lhs == rhs);
}

#endif  // RMF_SCHEDULER__TASK__ESTIMATE_INTERFACE_HPP_
