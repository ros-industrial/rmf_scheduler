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

#include "rmf2_scheduler/utils/url_utils.hpp"

namespace rmf2_scheduler
{

namespace url
{

namespace
{

// Given a URL string, determine where the query string starts and ends.
// URLs have schema, domain and path (along with possible user name, password
// and port number which are of no interest for us here) which could optionally
// have a query string that is separated from the path by '?'. Finally, the URL
// could also have a '#'-separated URL fragment which is usually used by the
// browser as a bookmark element. So, for example:
//    http://server.com/path/to/object?k=v&foo=bar#fragment
// Here:
//    http://server.com/path/to/object - is the URL of the object,
//    ?k=v&foo=bar                     - URL query string
//    #fragment                        - URL fragment string
// If |exclude_fragment| is true, the function returns the start character and
// the length of the query string alone. If it is false, the query string length
// will include both the query string and the fragment.
bool get_query_string_pos(
  const std::string & url,
  bool exclude_fragment,
  size_t * query_pos,
  size_t * query_len)
{
  size_t query_start = url.find_first_of("?#");
  if (query_start == std::string::npos) {
    *query_pos = url.size();
    if (query_len) {
      *query_len = 0;
    }
    return false;
  }
  *query_pos = query_start;
  if (query_len) {
    size_t query_end = url.size();
    if (exclude_fragment) {
      if (url[query_start] == '?') {
        size_t pos_fragment = url.find('#', query_start);
        if (pos_fragment != std::string::npos) {
          query_end = pos_fragment;
        }
      } else {
        query_end = query_start;
      }
    }
    *query_len = query_end - query_start;
  }
  return true;
}

}  // namespace

std::string combine(const std::string & url, const std::string & subpath)
{
  return combine_multiple(url, {subpath});
}

std::string combine_multiple(
  const std::string & url,
  const std::vector<std::string> & parts)
{
  std::string result = url;
  if (!parts.empty()) {
    std::string query_string = trim_off_query_string(&result);
    for (const auto & part : parts) {
      if (!part.empty()) {
        if (!result.empty() && result.back() != '/') {
          result += '/';
        }
        size_t non_slash_pos = part.find_first_not_of('/');
        if (non_slash_pos != std::string::npos) {
          result += data_encoding::url_encode(part.substr(non_slash_pos).c_str());
        }
      }
    }
    result += query_string;
  }
  return result;
}

std::string trim_off_query_string(std::string * url)
{
  size_t query_pos;
  if (!get_query_string_pos(*url, false, &query_pos, nullptr)) {
    return std::string();
  }
  std::string query_string = url->substr(query_pos);
  url->resize(query_pos);
  return query_string;
}

std::string get_query_string(const std::string & url, bool remove_fragment)
{
  std::string query_string;
  size_t query_pos, query_len;
  if (get_query_string_pos(url, remove_fragment, &query_pos, &query_len)) {
    query_string = url.substr(query_pos, query_len);
  }
  return query_string;
}

data_encoding::WebParamList get_query_string_parameters(
  const std::string & url
)
{
  // Extract the query string and remove the leading '?'.
  std::string query_string = get_query_string(url, true);
  if (!query_string.empty() && query_string.front() == '?') {
    query_string.erase(query_string.begin());
  }
  return data_encoding::web_params_decode(query_string);
}

std::string get_query_string_value(
  const std::string & url,
  const std::string & name
)
{
  return get_query_string_value(get_query_string_parameters(url), name);
}

std::string get_query_string_value(
  const data_encoding::WebParamList & params,
  const std::string & name)
{
  for (const auto & pair : params) {
    if (name.compare(pair.first) == 0) {
      return pair.second;
    }
  }
  return std::string();
}

std::string remove_query_string(
  const std::string & url,
  bool remove_fragment_too
)
{
  size_t query_pos, query_len;
  if (!get_query_string_pos(url, !remove_fragment_too, &query_pos, &query_len)) {
    return url;
  }
  std::string result = url.substr(0, query_pos);
  size_t fragment_pos = query_pos + query_len;
  if (fragment_pos < url.size()) {
    result += url.substr(fragment_pos);
  }
  return result;
}

std::string append_query_param(
  const std::string & url,
  const std::string & name,
  const std::string & value
)
{
  return append_query_params(url, {{name, value}});
}

std::string append_query_params(
  const std::string & url,
  const data_encoding::WebParamList & params
)
{
  if (params.empty()) {
    return url;
  }
  size_t query_pos, query_len;
  get_query_string_pos(url, true, &query_pos, &query_len);
  size_t fragment_pos = query_pos + query_len;
  std::string result = url.substr(0, fragment_pos);
  if (query_len == 0) {
    result += '?';
  } else if (query_len > 1) {
    result += '&';
  }
  result += data_encoding::web_params_encode(params);
  if (fragment_pos < url.size()) {
    result += url.substr(fragment_pos);
  }
  return result;
}

bool has_query_string(const std::string & url)
{
  size_t query_pos, query_len;
  get_query_string_pos(url, true, &query_pos, &query_len);
  return query_len > 0;
}

}  // namespace url
}  // namespace rmf2_scheduler
