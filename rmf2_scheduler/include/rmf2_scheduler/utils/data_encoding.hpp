// Copyright 2025 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__UTILS__DATA_ENCODING_HPP_
#define RMF2_SCHEDULER__UTILS__DATA_ENCODING_HPP_

#include <string>
#include <utility>
#include <vector>

namespace rmf2_scheduler
{

namespace data_encoding
{

using WebParamList = std::vector<std::pair<std::string, std::string>>;

// Encode/escape string to be used in the query portion of a URL.
// If |encodeSpaceAsPlus| is set to true, spaces are encoded as '+' instead
// of "%20"
std::string url_encode(const char * data, bool encode_space_as_plus);

inline std::string url_encode(const char * data)
{
  return url_encode(data, true);
}
// Decodes/unescapes a URL. Replaces all %XX sequences with actual characters.
// Also replaces '+' with spaces.
std::string url_decode(const char * data);

// Converts a list of key-value pairs into a string compatible with
// 'application/x-www-form-urlencoded' content encoding.
std::string web_params_encode(
  const WebParamList & params,
  bool encodeSpaceAsPlus
);

inline std::string web_params_encode(const WebParamList & params)
{
  return web_params_encode(params, true);
}

// Parses a string of '&'-delimited key-value pairs (separated by '=') and
// encoded in a way compatible with 'application/x-www-form-urlencoded'
// content encoding.
WebParamList web_params_decode(const std::string & data);

}  // namespace data_encoding
}  // namespace rmf2_scheduler


#endif  // RMF2_SCHEDULER__UTILS__DATA_ENCODING_HPP_
