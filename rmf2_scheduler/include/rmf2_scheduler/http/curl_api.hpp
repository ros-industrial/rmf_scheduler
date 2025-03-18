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
// +/refs/heads/upstream/brillo/http/curl_api.h

#ifndef RMF2_SCHEDULER__HTTP__CURL_API_HPP_
#define RMF2_SCHEDULER__HTTP__CURL_API_HPP_

#include <string>

#include "curl/curl.h"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace http
{

/// Abstract wrapper around libcurl C API that allows us to mock it out in tests.
class CURLInterface
{
public:
  CURLInterface() = default;
  virtual ~CURLInterface() = default;

  // Wrapper around curl_easy_init().
  virtual CURL * easy_init() = 0;

  // Wrapper around curl_easy_cleanup().
  virtual void easy_cleanup(CURL * curl) = 0;

  // Wrappers around curl_easy_setopt().
  virtual CURLcode easy_setopt_int(CURL * curl, CURLoption option, int value) = 0;
  virtual CURLcode easy_setopt_str(
    CURL * curl,
    CURLoption option,
    const std::string & value) = 0;
  virtual CURLcode easy_setopt_ptr(
    CURL * curl,
    CURLoption option,
    void * value) = 0;
  virtual CURLcode easy_setopt_callback(
    CURL * curl,
    CURLoption option,
    intptr_t address) = 0;
  virtual CURLcode easy_setopt_off_t(
    CURL * curl,
    CURLoption option,
    curl_off_t value) = 0;

  // A type-safe wrapper around function callback options.
  template<typename R, typename ... Args>
  inline CURLcode easy_setopt_callback(
    CURL * curl,
    CURLoption option,
    R (* callback)(Args...))
  {
    return easy_setopt_callback(
      curl, option, reinterpret_cast<intptr_t>(callback));
  }

  // Wrapper around curl_easy_perform().
  virtual CURLcode easy_perform(CURL * curl) = 0;

  // Wrappers around curl_easy_getinfo().
  virtual CURLcode easy_getinfo_int(
    CURL * curl,
    CURLINFO info,
    int * value) const = 0;
  virtual CURLcode easy_getinfo_dbl(
    CURL * curl,
    CURLINFO info,
    double * value) const = 0;
  virtual CURLcode easy_getinfo_str(
    CURL * curl,
    CURLINFO info,
    std::string * value) const = 0;
  virtual CURLcode easy_getinfo_ptr(
    CURL * curl,
    CURLINFO info,
    void ** value) const = 0;

  // Wrapper around curl_easy_strerror().
  virtual std::string easy_strerror(CURLcode code) const = 0;

  // Wrapper around curl_multi_init().
  virtual CURLM * multi_init() = 0;

  // Wrapper around curl_multi_cleanup().
  virtual CURLMcode multi_cleanup(CURLM * multi_handle) = 0;

  // Wrapper around curl_multi_info_read().
  virtual CURLMsg * multi_info_read(CURLM * multi_handle, int * msgs_in_queue) = 0;

  // Wrapper around curl_multi_add_handle().
  virtual CURLMcode multi_add_handle(CURLM * multi_handle, CURL * curl_handle) = 0;

  // Wrapper around curl_multi_remove_handle().
  virtual CURLMcode multi_remove_handle(
    CURLM * multi_handle,
    CURL * curl_handle) = 0;

  // Wrapper around curl_multi_setopt(CURLMOPT_SOCKETFUNCTION/SOCKETDATA).
  virtual CURLMcode multi_set_socket_callback(
    CURLM * multi_handle,
    curl_socket_callback socket_callback,
    void * userp) = 0;

  // Wrapper around curl_multi_setopt(CURLMOPT_TIMERFUNCTION/TIMERDATA).
  virtual CURLMcode multi_set_timer_callback(
    CURLM * multi_handle,
    curl_multi_timer_callback timer_callback,
    void * userp) = 0;

  // Wrapper around curl_multi_assign().
  virtual CURLMcode multi_assign(
    CURLM * multi_handle,
    curl_socket_t sockfd,
    void * sockp) = 0;

  // Wrapper around curl_multi_socket_action().
  virtual CURLMcode multi_socket_action(
    CURLM * multi_handle,
    curl_socket_t s,
    int ev_bitmask,
    int * running_handles) = 0;
  // Wrapper around curl_multi_strerror().
  virtual std::string multi_strerror(CURLMcode code) const = 0;

private:
  RS_DISABLE_COPY(CURLInterface)
};

class CURLApi : public CURLInterface
{
public:
  CURLApi();
  ~CURLApi() override;

  // Wrapper around curl_easy_init().
  CURL * easy_init() override;

  // Wrapper around curl_easy_cleanup().
  void easy_cleanup(CURL * curl) override;

  // Wrappers around curl_easy_setopt().
  CURLcode easy_setopt_int(CURL * curl, CURLoption option, int value) override;
  CURLcode easy_setopt_str(
    CURL * curl,
    CURLoption option,
    const std::string & value) override;
  CURLcode easy_setopt_ptr(CURL * curl, CURLoption option, void * value) override;
  CURLcode easy_setopt_callback(
    CURL * curl,
    CURLoption option,
    intptr_t address) override;
  CURLcode easy_setopt_off_t(
    CURL * curl,
    CURLoption option,
    curl_off_t value) override;

  // Wrapper around curl_easy_perform().
  CURLcode easy_perform(CURL * curl) override;

  // Wrappers around curl_easy_getinfo().
  CURLcode easy_getinfo_int(CURL * curl, CURLINFO info, int * value) const override;
  CURLcode easy_getinfo_dbl(
    CURL * curl,
    CURLINFO info,
    double * value) const override;
  CURLcode easy_getinfo_str(
    CURL * curl,
    CURLINFO info,
    std::string * value) const override;
  CURLcode easy_getinfo_ptr(
    CURL * curl,
    CURLINFO info,
    void ** value) const override;

  // Wrapper around curl_easy_strerror().
  std::string easy_strerror(CURLcode code) const override;

  // Wrapper around curl_multi_init().
  CURLM * multi_init() override;

  // Wrapper around curl_multi_cleanup().
  CURLMcode multi_cleanup(CURLM * multi_handle) override;

  // Wrapper around curl_multi_info_read().
  CURLMsg * multi_info_read(CURLM * multi_handle, int * msgs_in_queue) override;

  // Wrapper around curl_multi_add_handle().
  CURLMcode multi_add_handle(CURLM * multi_handle, CURL * curl_handle) override;

  // Wrapper around curl_multi_remove_handle().
  CURLMcode multi_remove_handle(CURLM * multi_handle, CURL * curl_handle) override;

  // Wrapper around curl_multi_setopt(CURLMOPT_SOCKETFUNCTION/SOCKETDATA).
  CURLMcode multi_set_socket_callback(
    CURLM * multi_handle,
    curl_socket_callback socket_callback,
    void * userp) override;

  // Wrapper around curl_multi_setopt(CURLMOPT_TIMERFUNCTION/TIMERDATA).
  CURLMcode multi_set_timer_callback(
    CURLM * multi_handle,
    curl_multi_timer_callback timer_callback,
    void * userp) override;

  // Wrapper around curl_multi_assign().
  CURLMcode multi_assign(
    CURLM * multi_handle,
    curl_socket_t sockfd,
    void * sockp) override;

  // Wrapper around curl_multi_socket_action().
  CURLMcode multi_socket_action(
    CURLM * multi_handle,
    curl_socket_t s,
    int ev_bitmask,
    int * running_handles) override;

  // Wrapper around curl_multi_strerror().
  std::string multi_strerror(CURLMcode code) const override;

private:
  RS_DISABLE_COPY(CURLApi)
};

}  // namespace http
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__CURL_API_HPP_
