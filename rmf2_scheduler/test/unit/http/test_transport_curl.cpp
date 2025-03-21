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

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "rmf2_scheduler/http/mock_curl_api.hpp"
#include "rmf2_scheduler/http/transport_curl.hpp"
#include "rmf2_scheduler/log.hpp"

class TestHTTPTransportCURL : public ::testing::Test
{
public:
  void SetUp() override
  {
    using testing::_;
    using testing::Return;
    using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

    curl_api_ = std::make_shared<MockCURLInterface>();
    transport_ = std::make_shared<curl::Transport>(curl_api_);
    handle_ = reinterpret_cast<CURL *>(100);  // Mock handle value.
    EXPECT_CALL(*curl_api_, easy_init()).WillOnce(Return(handle_));

    // TODO(anyone): CA certificate
    // EXPECT_CALL(*curl_api_, easy_setopt_ptr(handle_, CURLOPT_CAINFO, _))
    //   .WillOnce(Return(CURLE_OK));
    // EXPECT_CALL(*curl_api_, easy_setopt_str(handle_, CURLOPT_CAPATH, _))
    //   .WillOnce(Return(CURLE_OK));

    EXPECT_CALL(*curl_api_, easy_setopt_int(handle_, CURLOPT_SSL_VERIFYPEER, 1))
    .WillOnce(Return(CURLE_OK));
    EXPECT_CALL(*curl_api_, easy_setopt_int(handle_, CURLOPT_SSL_VERIFYHOST, 2))
    .WillOnce(Return(CURLE_OK));
    EXPECT_CALL(*curl_api_, easy_setopt_ptr(handle_, CURLOPT_PRIVATE, _))
    .WillRepeatedly(Return(CURLE_OK));
  }

  void TearDown() override
  {
    transport_.reset();
    curl_api_.reset();
  }

protected:
  std::shared_ptr<rmf2_scheduler::http::MockCURLInterface> curl_api_;
  std::shared_ptr<rmf2_scheduler::http::curl::Transport> transport_;
  CURL * handle_ = nullptr;
};

TEST_F(TestHTTPTransportCURL, request_GET) {
  using testing::Return;
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_URL, "http://foo.bar/get")
  ).WillOnce(Return(CURLE_OK));

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_USERAGENT, "User Agent")
  ).WillOnce(Return(CURLE_OK));

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_REFERER, "http://foo.bar/baz")
  ).WillOnce(Return(CURLE_OK));

  EXPECT_CALL(*curl_api_, easy_setopt_int(handle_, CURLOPT_HTTPGET, 1))
  .WillOnce(Return(CURLE_OK));

  std::string error;
  auto connection = transport_->create_connection(
    "http://foo.bar/get",
    request_type::kGet,
    {},
    "User Agent",
    "http://foo.bar/baz",
    error
  );

  EXPECT_NE(nullptr, connection.get());
  EXPECT_CALL(*curl_api_, easy_cleanup(handle_)).Times(1);
  connection.reset();
}

TEST_F(TestHTTPTransportCURL, request_GET_with_proxy) {
  using testing::Return;
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_URL, "http://foo.bar/get")
  ).WillOnce(Return(CURLE_OK));

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_USERAGENT, "User Agent")
  ).WillOnce(Return(CURLE_OK));

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_REFERER, "http://foo.bar/baz")
  ).WillOnce(Return(CURLE_OK));

  EXPECT_CALL(*curl_api_, easy_setopt_int(handle_, CURLOPT_HTTPGET, 1))
  .WillOnce(Return(CURLE_OK));

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_PROXY, "http://proxy.server")
  ).WillOnce(Return(CURLE_OK));

  std::string error;
  std::shared_ptr<curl::Transport> proxy_transport =
    std::make_shared<curl::Transport>(curl_api_, "http://proxy.server");
  auto connection = proxy_transport->create_connection(
    "http://foo.bar/get",
    request_type::kGet,
    {},
    "User Agent",
    "http://foo.bar/baz",
    error
  );
  EXPECT_NE(nullptr, connection.get());
  EXPECT_CALL(*curl_api_, easy_cleanup(handle_)).Times(1);
  connection.reset();
}

TEST_F(TestHTTPTransportCURL, request_HEAD) {
  using testing::Return;
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_URL, "http://foo.bar/head")
  ).WillOnce(Return(CURLE_OK));
  EXPECT_CALL(*curl_api_, easy_setopt_int(handle_, CURLOPT_NOBODY, 1))
  .WillOnce(Return(CURLE_OK));

  std::string error;
  auto connection = transport_->create_connection(
    "http://foo.bar/head",
    request_type::kHead,
    {},
    "",
    "",
    error
  );
  EXPECT_NE(nullptr, connection.get());
  EXPECT_CALL(*curl_api_, easy_cleanup(handle_)).Times(1);
  connection.reset();
}

TEST_F(TestHTTPTransportCURL, request_PUT) {
  using testing::Return;
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_URL, "http://foo.bar/put")
  ).WillOnce(Return(CURLE_OK));
  EXPECT_CALL(*curl_api_, easy_setopt_int(handle_, CURLOPT_UPLOAD, 1))
  .WillOnce(Return(CURLE_OK));

  std::string error;
  auto connection = transport_->create_connection(
    "http://foo.bar/put",
    request_type::kPut,
    {},
    "",
    "",
    error
  );
  EXPECT_NE(nullptr, connection.get());
  EXPECT_CALL(*curl_api_, easy_cleanup(handle_)).Times(1);
  connection.reset();
}

TEST_F(TestHTTPTransportCURL, request_POST) {
  using testing::Return;
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_URL, "http://foo.bar/post")
  ).WillOnce(Return(CURLE_OK));
  EXPECT_CALL(*curl_api_, easy_setopt_int(handle_, CURLOPT_POST, 1))
  .WillOnce(Return(CURLE_OK));
  EXPECT_CALL(*curl_api_, easy_setopt_ptr(handle_, CURLOPT_POSTFIELDS, nullptr))
  .WillOnce(Return(CURLE_OK));

  std::string error;
  auto connection = transport_->create_connection(
    "http://foo.bar/post",
    request_type::kPost,
    {},
    "",
    "",
    error
  );
  EXPECT_NE(nullptr, connection.get());
  EXPECT_CALL(*curl_api_, easy_cleanup(handle_)).Times(1);
  connection.reset();
}

TEST_F(TestHTTPTransportCURL, request_PATCH) {
  using testing::Return;
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_URL, "http://foo.bar/patch")
  ).WillOnce(Return(CURLE_OK));
  EXPECT_CALL(*curl_api_, easy_setopt_int(handle_, CURLOPT_POST, 1))
  .WillOnce(Return(CURLE_OK));
  EXPECT_CALL(*curl_api_, easy_setopt_ptr(handle_, CURLOPT_POSTFIELDS, nullptr))
  .WillOnce(Return(CURLE_OK));
  EXPECT_CALL(*curl_api_, easy_setopt_str(handle_, CURLOPT_CUSTOMREQUEST, request_type::kPatch))
  .WillOnce(Return(CURLE_OK));

  std::string error;
  auto connection = transport_->create_connection(
    "http://foo.bar/patch",
    request_type::kPatch,
    {},
    "",
    "",
    error
  );
  EXPECT_NE(nullptr, connection.get());
  EXPECT_CALL(*curl_api_, easy_cleanup(handle_)).Times(1);
  connection.reset();
}

TEST_F(TestHTTPTransportCURL, curl_failure) {
  using testing::Return;
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  EXPECT_CALL(
    *curl_api_,
    easy_setopt_str(handle_, CURLOPT_URL, "http://foo.bar/get")
  ).WillOnce(Return(CURLE_OK));
  EXPECT_CALL(*curl_api_, easy_setopt_int(handle_, CURLOPT_HTTPGET, 1))
  .WillOnce(Return(CURLE_OUT_OF_MEMORY));
  EXPECT_CALL(*curl_api_, easy_strerror(CURLE_OUT_OF_MEMORY))
  .WillOnce(Return("Out of Memory"));
  EXPECT_CALL(*curl_api_, easy_cleanup(handle_)).Times(1);
  std::string error;
  auto connection = transport_->create_connection(
    "http://foo.bar/get",
    request_type::kGet,
    {},
    "",
    "",
    error
  );
  EXPECT_EQ(nullptr, connection.get());
  EXPECT_EQ("Transport create_connection failed, curl error: Out of Memory", error);
}

// TODO(anyone): DNS and more
