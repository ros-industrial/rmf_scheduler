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

#include "rmf_notification/message.hpp"

namespace rmf_notification
{

Message::Message(
  std::string _type,
  std::string _payload,
  uint64_t _timestamp)
: type(_type),
  payload(_payload),
  timestamp(_timestamp)
{
}

}  // namespace rmf_notification
