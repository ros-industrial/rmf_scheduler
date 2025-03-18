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

#include <cassert>

#include "rmf2_scheduler/utils/data_encoding.hpp"
#include "rmf2_scheduler/utils/string_utils.hpp"

namespace rmf2_scheduler
{

namespace data_encoding
{

namespace
{

inline int hex_to_dec(int hex)
{
  int dec = -1;
  if (hex >= '0' && hex <= '9') {
    dec = hex - '0';
  } else if (hex >= 'A' && hex <= 'F') {
    dec = hex - 'A' + 10;
  } else if (hex >= 'a' && hex <= 'f') {
    dec = hex - 'a' + 10;
  }
  return dec;
}

inline void string_append_f(
  std::string * out,
  const char * format,
  unsigned char c
)
{
  static char buffer[4];
  int cx = snprintf(buffer, sizeof(buffer), format, c);
  assert(cx > 0 && cx < 4);
  *out += std::string(buffer);
}

}  // namespace

std::string url_encode(
  const char * data,
  bool encode_space_as_plus
)
{
  std::string result;
  while (*data) {
    char c = *data++;
    // According to RFC3986 (http://www.faqs.org/rfcs/rfc3986.html),
    // section 2.3. - Unreserved Characters
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') ||
      (c >= 'a' && c <= 'z') || c == '-' || c == '.' || c == '_' ||
      c == '~')
    {
      result += c;
    } else if (c == ' ' && encode_space_as_plus) {
      // For historical reasons, some URLs have spaces encoded as '+',
      // this also applies to form data encoded as
      // 'application/x-www-form-urlencoded'
      result += '+';
    } else {
      string_append_f(&result, "%%%02X", static_cast<unsigned char>(c));  // Encode as %NN
    }
  }
  return result;
}

std::string url_decode(const char * data)
{
  std::string result;
  while (*data) {
    char c = *data++;
    int part1 = 0, part2 = 0;
    // HexToDec would return -1 even for character 0 (end of string),
    // so it is safe to access data[0] and data[1] without overrunning the buf.
    if (c == '%' && (part1 = hex_to_dec(data[0])) >= 0 &&
      (part2 = hex_to_dec(data[1])) >= 0)
    {
      c = static_cast<char>((part1 << 4) | part2);
      data += 2;
    } else if (c == '+') {
      c = ' ';
    }
    result += c;
  }
  return result;
}

std::string web_params_encode(
  const WebParamList & params,
  bool encodeSpaceAsPlus
)
{
  std::vector<std::string> pairs;
  pairs.reserve(params.size());
  for (const auto & p : params) {
    std::string key = url_encode(p.first.c_str(), encodeSpaceAsPlus);
    std::string value = url_encode(p.second.c_str(), encodeSpaceAsPlus);
    pairs.push_back(string_utils::join("=", key, value));
  }
  return string_utils::join("&", pairs);
}

WebParamList web_params_decode(const std::string & data)
{
  WebParamList result;
  std::vector<std::string> params = string_utils::split(data, "&");
  for (const auto & p : params) {
    auto pair = string_utils::split_at_first(p, "=");
    result.emplace_back(
      url_decode(pair.first.c_str()),
      url_decode(pair.second.c_str()));
  }
  return result;
}

}  // namespace data_encoding
}  // namespace rmf2_scheduler
