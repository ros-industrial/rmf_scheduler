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

#ifndef RMF2_SCHEDULER__UTILS__URL_UTILS_HPP_
#define RMF2_SCHEDULER__UTILS__URL_UTILS_HPP_

#include <string>
#include <vector>

#include "rmf2_scheduler/utils/data_encoding.hpp"

namespace rmf2_scheduler
{

namespace url
{

/// Appends a subpath to url and delimiting then with '/'.
/**
 * This also handles URLs with query parameters/fragment.
 */
[[nodiscard]] std::string combine(
  const std::string & url,
  const std::string & subpath
);

[[nodiscard]] std::string combine_multiple(
  const std::string & url,
  const std::vector<std::string> & parts
);

/// Removes the query string/fragment from |url| and returns the query string.
/**
 * This method actually modifies |url|. So, if you call it on this:
 *   http://www.test.org/?foo=bar
 * it will modify |url| to "http://www.test.org/" and return "?foo=bar"
 */
std::string trim_off_query_string(std::string * url);

/// Returns the query string, if available.
/**
 * For example, for the following URL:
 *   http://server.com/path/to/object?k=v&foo=bar#fragment
 * Here:
 *   http://server.com/path/to/object - is the URL of the object,
 *   ?k=v&foo=bar                     - URL query string
 *    #fragment                        - URL fragment string
 * If |remove_fragment| is true, the function returns the query string without
 * the fragment. Otherwise the fragment is included as part of the result.
 */
std::string get_query_string(
  const std::string & url,
  bool remove_fragment
);

/// Parses the query string into a set of key-value pairs.
data_encoding::WebParamList get_query_string_parameters(
  const std::string & url
);

/// Returns a value of the specified query parameter, or empty string if missing.
std::string get_query_string_value(
  const std::string & url,
  const std::string & name
);

std::string get_query_string_value(
  const data_encoding::WebParamList & params,
  const std::string & name
);

/// Removes the query string and/or a fragment part from URL.
/** If |remove_fragment| is specified, the fragment is also removed.
 * For example:
 *   http://server.com/path/to/object?k=v&foo=bar#fragment
 * true  -> http://server.com/path/to/object
 * false -> http://server.com/path/to/object#fragment
 */
[[nodiscard]] std::string remove_query_string(
  const std::string & url,
  bool remove_fragment
);

/// Appends a single query parameter to the URL.
[[nodiscard]] std::string append_query_param(
  const std::string & url,
  const std::string & name,
  const std::string & value
);

/// Appends a list of query parameters to the URL.
[[nodiscard]] std::string append_query_params(
  const std::string & url,
  const data_encoding::WebParamList & params
);

/// Checks if the URL has query parameters.
bool has_query_string(const std::string & url);

}  // namespace url
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__UTILS__URL_UTILS_HPP_
