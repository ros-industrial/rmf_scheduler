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

#ifndef RMF_SCHEDULER__ESTIMATE_INTERFACE_HPP_
#define RMF_SCHEDULER__ESTIMATE_INTERFACE_HPP_

#include <future>
#include <string>
#include <unordered_set>

#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rmf_scheduler
{

class EstimateInterface
{
public:
  virtual ~EstimateInterface() {}

  virtual void init(
    const std::string & name,
    const std::string & ns)
  {
    node_ = rclcpp::Node::make_shared(name, ns);
  }

  virtual std::shared_future<nlohmann::json> async_estimate(
    const std::string & id,
    const nlohmann::json & event_details) = 0;

  virtual const std::unordered_set<std::string> & support_task_types() const = 0;

  const rclcpp::Node::SharedPtr & get_node() const
  {
    return node_;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__ESTIMATE_INTERFACE_HPP_
