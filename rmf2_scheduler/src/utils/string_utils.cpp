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

#include "rmf2_scheduler/utils/string_utils.hpp"

namespace rmf2_scheduler
{

namespace string_utils
{

namespace
{

inline void trim_whitespaces(
  const std::string & input,
  std::string * output
)
{
  // find the first and last good character
  const size_t first_good_char = input.find_first_not_of(' ');

  const size_t last_good_char = input.find_last_not_of(' ');

  // Empty input or no good character
  if (input.empty() || first_good_char == std::string::npos ||
    last_good_char == std::string::npos)
  {
    output->clear();
  }

  // Trim
  output->assign(input.data() + first_good_char, last_good_char - first_good_char + 1);
}

}  // namespace

std::vector<std::string> split(
  const std::string & str,
  const std::string & delimiter,
  bool _trim_whitespaces,
  bool purge_empty_strings
)
{
  std::vector<std::string> tokens;
  if (str.empty()) {
    return tokens;
  }
  for (size_t i = 0;; ) {
    const size_t pos = delimiter.empty() ? (i + 1) : str.find(delimiter, i);
    std::string tmp_str{str.substr(i, pos - i)};
    if (_trim_whitespaces) {
      trim_whitespaces(tmp_str, &tmp_str);
    }
    if (!tmp_str.empty() || !purge_empty_strings) {
      tokens.emplace_back(std::move(tmp_str));
    }
    if (pos >= str.size()) {
      break;
    }
    i = pos + delimiter.size();
  }
  return tokens;
}

std::pair<std::string, std::string> split_at_first(
  const std::string & str,
  const std::string & delimiter,
  bool _trim_whitespaces
)
{
  std::pair<std::string, std::string> pair;
  size_t pos = str.find(delimiter);
  if (pos != std::string::npos) {
    pair.first = str.substr(0, pos);
    pair.second = str.substr(pos + delimiter.size());
  } else {
    pair.first = str;
  }

  if (_trim_whitespaces) {
    trim_whitespaces(pair.first, &pair.first);
    trim_whitespaces(pair.second, &pair.second);
  }

  return pair;
}

static char to_lower_ascii(char c)
{
  return (c >= 'A' && c <= 'Z') ? (c + ('a' - 'A')) : c;
}

std::string to_lower_ascii(
  const std::string & str
)
{
  std::string out;
  out.reserve(str.size());
  for (size_t i = 0; i < str.size(); i++) {
    out.push_back(to_lower_ascii(str[i]));
  }

  return out;
}

}  // namespace string_utils
}  // namespace rmf2_scheduler
