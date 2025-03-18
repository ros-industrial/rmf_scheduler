// Copyright 2024 ROS Industrial Consortium Asia Pacific
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
//
// Original file taken from:
// https://chromium.googlesource.com/aosp/platform/external/libchromeos/
// +/refs/heads/upstream/brillo/http/curl_api.cc

#include <stdexcept>

#include "rmf2_scheduler/http/curl_api.hpp"

namespace rmf2_scheduler
{

namespace http
{

namespace
{

static_assert(
  CURLOPTTYPE_LONG == 0 &&
  CURLOPTTYPE_OBJECTPOINT == 10000 &&
  CURLOPTTYPE_FUNCTIONPOINT == 20000 &&
  CURLOPTTYPE_OFF_T == 30000,
  "CURL option types are expected to be multiples of 10000");

inline bool VerifyOptionType(CURLoption option, int expected_type)
{
  int option_type = (static_cast<int>(option) / 10000) * 10000;
  return option_type == expected_type;
}

}  // namespace

CURLApi::CURLApi()
{
  curl_global_init(CURL_GLOBAL_ALL);
}

CURLApi::~CURLApi()
{
  curl_global_cleanup();
}

CURL * CURLApi::easy_init()
{
  return curl_easy_init();
}

void CURLApi::easy_cleanup(CURL * curl)
{
  curl_easy_cleanup(curl);
}

CURLcode CURLApi::easy_setopt_int(CURL * curl, CURLoption option, int value)
{
  if (!VerifyOptionType(option, CURLOPTTYPE_LONG)) {
    throw std::invalid_argument(
            "Only options that expect a LONG data type must be specified here"
    );
  }
  // CURL actually uses "long" type, so have to make sure we feed it what it
  // expects.
  // NOLINTNEXTLINE(runtime/int)
  return curl_easy_setopt(curl, option, static_cast<long>(value));
}

CURLcode CURLApi::easy_setopt_str(
  CURL * curl,
  CURLoption option,
  const std::string & value)
{
  if (!VerifyOptionType(option, CURLOPTTYPE_OBJECTPOINT)) {
    throw std::invalid_argument(
            "Only options that expect a STRING data type must be specified here"
    );
  }
  return curl_easy_setopt(curl, option, value.c_str());
}

CURLcode CURLApi::easy_setopt_ptr(CURL * curl, CURLoption option, void * value)
{
  if (!VerifyOptionType(option, CURLOPTTYPE_OBJECTPOINT)) {
    throw std::invalid_argument(
            "Only options that expect a pointer data type must be specified here"
    );
  }
  return curl_easy_setopt(curl, option, value);
}

CURLcode CURLApi::easy_setopt_callback(
  CURL * curl,
  CURLoption option,
  intptr_t address)
{
  if (!VerifyOptionType(option, CURLOPTTYPE_FUNCTIONPOINT)) {
    throw std::invalid_argument(
            "Only options that expect a function pointers must be specified here"
    );
  }
  return curl_easy_setopt(curl, option, address);
}

CURLcode CURLApi::easy_setopt_off_t(
  CURL * curl,
  CURLoption option,
  curl_off_t value)
{
  if (!VerifyOptionType(option, CURLOPTTYPE_OFF_T)) {
    throw std::invalid_argument(
            "Only options that expect a large data size must be specified here"
    );
  }
  return curl_easy_setopt(curl, option, value);
}

CURLcode CURLApi::easy_perform(CURL * curl)
{
  return curl_easy_perform(curl);
}

CURLcode CURLApi::easy_getinfo_int(CURL * curl, CURLINFO info, int * value) const
{
  if (CURLINFO_LONG != (info & CURLINFO_TYPEMASK)) {
    throw std::invalid_argument("Wrong option type");
  }
  long data = 0;  // NOLINT(runtime/int) - curl expects a long here.
  CURLcode code = curl_easy_getinfo(curl, info, &data);
  if (code == CURLE_OK) {
    *value = static_cast<int>(data);
  }
  return code;
}

CURLcode CURLApi::easy_getinfo_dbl(
  CURL * curl,
  CURLINFO info,
  double * value) const
{
  if (CURLINFO_DOUBLE != (info & CURLINFO_TYPEMASK)) {
    throw std::invalid_argument("Wrong option type");
  }
  return curl_easy_getinfo(curl, info, value);
}

CURLcode CURLApi::easy_getinfo_str(
  CURL * curl,
  CURLINFO info,
  std::string * value) const
{
  if (CURLINFO_STRING != (info & CURLINFO_TYPEMASK)) {
    std::invalid_argument("Wrong option type");
  }
  char * data = nullptr;
  CURLcode code = curl_easy_getinfo(curl, info, &data);
  if (code == CURLE_OK) {
    *value = data;
  }
  return code;
}

CURLcode CURLApi::easy_getinfo_ptr(
  CURL * curl,
  CURLINFO info,
  void ** value) const
{
  // CURL uses "string" type for generic pointer info. Go figure.
  if (CURLINFO_STRING != (info & CURLINFO_TYPEMASK)) {
    std::invalid_argument("Wrong option type");
  }
  return curl_easy_getinfo(curl, info, value);
}

std::string CURLApi::easy_strerror(CURLcode code) const
{
  return curl_easy_strerror(code);
}

CURLM * CURLApi::multi_init()
{
  return curl_multi_init();
}

CURLMcode CURLApi::multi_cleanup(CURLM * multi_handle)
{
  return curl_multi_cleanup(multi_handle);
}

CURLMsg * CURLApi::multi_info_read(CURLM * multi_handle, int * msgs_in_queue)
{
  return curl_multi_info_read(multi_handle, msgs_in_queue);
}

CURLMcode CURLApi::multi_add_handle(CURLM * multi_handle, CURL * curl_handle)
{
  return curl_multi_add_handle(multi_handle, curl_handle);
}

CURLMcode CURLApi::multi_remove_handle(CURLM * multi_handle, CURL * curl_handle)
{
  return curl_multi_remove_handle(multi_handle, curl_handle);
}

CURLMcode CURLApi::multi_set_socket_callback(
  CURLM * multi_handle,
  curl_socket_callback socket_callback,
  void * userp)
{
  CURLMcode code =
    curl_multi_setopt(multi_handle, CURLMOPT_SOCKETFUNCTION, socket_callback);
  if (code != CURLM_OK) {
    return code;
  }
  return curl_multi_setopt(multi_handle, CURLMOPT_SOCKETDATA, userp);
}

CURLMcode CURLApi::multi_set_timer_callback(
  CURLM * multi_handle,
  curl_multi_timer_callback timer_callback,
  void * userp)
{
  CURLMcode code =
    curl_multi_setopt(multi_handle, CURLMOPT_TIMERFUNCTION, timer_callback);
  if (code != CURLM_OK) {
    return code;
  }
  return curl_multi_setopt(multi_handle, CURLMOPT_TIMERDATA, userp);
}

CURLMcode CURLApi::multi_assign(
  CURLM * multi_handle,
  curl_socket_t sockfd,
  void * sockp)
{
  return curl_multi_assign(multi_handle, sockfd, sockp);
}

CURLMcode CURLApi::multi_socket_action(
  CURLM * multi_handle,
  curl_socket_t s,
  int ev_bitmask,
  int * running_handles)
{
  return curl_multi_socket_action(multi_handle, s, ev_bitmask, running_handles);
}

std::string CURLApi::multi_strerror(CURLMcode code) const
{
  return curl_multi_strerror(code);
}

}  // namespace http
}  // namespace rmf2_scheduler
