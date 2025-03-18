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

#ifndef RMF2_SCHEDULER__UTILS__STRING_UTILS_HPP_
#define RMF2_SCHEDULER__UTILS__STRING_UTILS_HPP_

#include <string>
#include <utility>
#include <vector>

namespace rmf2_scheduler
{

namespace string_utils
{

/// Split string
/**
 * Treats the string as a delimited list of substrings and returns the array
 * of original elements of the list.
 *
 * |trim_whitespaces| causes each element to have all whitespaces trimmed off.
 * |purge_empty_strings| specifies whether empty elements from the original
 *
 * string should be omitted.
 */
std::vector<std::string> split(
  const std::string & str,
  const std::string & delimiter,
  bool trim_whitespaces,
  bool purge_empty_strings
);

/// Split string
/**
 * Splits the string, trims all whitespaces, omits empty string parts.
 */
inline std::vector<std::string> split(
  const std::string & str,
  const std::string & delimiter)
{
  return split(str, delimiter, true, true);
}

/// Split string
/**
 * Splits the string, omits empty string parts.
 */
inline std::vector<std::string> split(
  const std::string & str,
  const std::string & delimiter,
  bool trim_whitespaces)
{
  return split(str, delimiter, trim_whitespaces, true);
}

/// Split the string into two pieces at the first position of the specified delimiter
std::pair<std::string, std::string> split_at_first(
  const std::string & str,
  const std::string & delimiter,
  bool trim_whitespaces = true);

/// Joins strings into a single string separated by |delimiter|.
template<class InputIterator>
std::string join_range(
  const std::string & delimiter,
  InputIterator first,
  InputIterator last
)
{
  std::string result;
  if (first == last) {
    return result;
  }
  result = *first;
  for (++first; first != last; ++first) {
    result += delimiter;
    result += *first;
  }
  return result;
}

template<class Container>
std::string join(const std::string & delimiter, const Container & strings)
{
  using std::begin;
  using std::end;
  return join_range(delimiter, begin(strings), end(strings));
}

inline std::string join(
  const std::string & delimiter,
  std::initializer_list<std::string> strings
)
{
  return join_range(delimiter, strings.begin(), strings.end());
}

inline std::string join(
  const std::string & delimiter,
  const std::string & str1,
  const std::string & str2
)
{
  return str1 + delimiter + str2;
}

std::string to_lower_ascii(
  const std::string & str
);

}  // namespace string_utils
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__UTILS__STRING_UTILS_HPP_
