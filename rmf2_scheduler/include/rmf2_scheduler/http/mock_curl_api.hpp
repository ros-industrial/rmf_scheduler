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


#ifndef RMF2_SCHEDULER__HTTP__MOCK_CURL_API_HPP_
#define RMF2_SCHEDULER__HTTP__MOCK_CURL_API_HPP_

#include <string>

#include "gmock/gmock.h"
#include "rmf2_scheduler/http/curl_api.hpp"

namespace rmf2_scheduler
{

namespace http
{

/// Abstract wrapper around libcurl C API that allows us to mock it out in tests.
class MockCURLInterface : public CURLInterface
{
public:
  MockCURLInterface() = default;
  MOCK_METHOD(CURL *, easy_init, (), (override));
  MOCK_METHOD(void, easy_cleanup, (CURL *), (override));
  MOCK_METHOD(CURLcode, easy_setopt_int, (CURL *, CURLoption, int), (override));
  MOCK_METHOD(
    CURLcode,
    easy_setopt_str,
    (CURL *, CURLoption, const std::string &),
    (override));
  MOCK_METHOD(CURLcode, easy_setopt_ptr, (CURL *, CURLoption, void *), (override));
  MOCK_METHOD(
    CURLcode,
    easy_setopt_callback,
    (CURL *, CURLoption, intptr_t),
    (override));
  MOCK_METHOD(
    CURLcode,
    easy_setopt_off_t,
    (CURL *, CURLoption, curl_off_t),
    (override));
  MOCK_METHOD(CURLcode, easy_perform, (CURL *), (override));
  MOCK_METHOD(
    CURLcode,
    easy_getinfo_int,
    (CURL *, CURLINFO, int *),
    (const, override));
  MOCK_METHOD(
    CURLcode,
    easy_getinfo_dbl,
    (CURL *, CURLINFO, double *),
    (const, override));
  MOCK_METHOD(
    CURLcode,
    easy_getinfo_str,
    (CURL *, CURLINFO, std::string *),
    (const, override));
  MOCK_METHOD(
    CURLcode,
    easy_getinfo_ptr,
    (CURL *, CURLINFO, void **),
    (const, override));
  MOCK_METHOD(std::string, easy_strerror, (CURLcode), (const, override));
  MOCK_METHOD(CURLM *, multi_init, (), (override));
  MOCK_METHOD(CURLMcode, multi_cleanup, (CURLM *), (override));
  MOCK_METHOD(CURLMsg *, multi_info_read, (CURLM *, int *), (override));
  MOCK_METHOD(CURLMcode, multi_add_handle, (CURLM *, CURL *), (override));
  MOCK_METHOD(CURLMcode, multi_remove_handle, (CURLM *, CURL *), (override));
  MOCK_METHOD(
    CURLMcode,
    multi_set_socket_callback,
    (CURLM *, curl_socket_callback, void *),
    (override));
  MOCK_METHOD(
    CURLMcode,
    multi_set_timer_callback,
    (CURLM *, curl_multi_timer_callback, void *),
    (override));
  MOCK_METHOD(
    CURLMcode,
    multi_assign,
    (CURLM *, curl_socket_t, void *),
    (override));
  MOCK_METHOD(
    CURLMcode,
    multi_socket_action,
    (CURLM *, curl_socket_t, int, int *),
    (override));
  MOCK_METHOD(std::string, multi_strerror, (CURLMcode), (const, override));
  // MOCK_METHOD(CURLMcode, multi_perform, (CURLM*, int*), (override));
  // MOCK_METHOD(CURLMcode,
  //             multi_wait,
  //             (CURLM*, curl_waitfd[], unsigned int, int, int*),
  //             (override));
};

}  // namespace http
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__MOCK_CURL_API_HPP_
