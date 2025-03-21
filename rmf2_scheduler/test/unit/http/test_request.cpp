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

#include "rmf2_scheduler/http/memory_stream.hpp"
#include "rmf2_scheduler/http/mock_connection.hpp"
#include "rmf2_scheduler/http/mock_transport.hpp"
#include "rmf2_scheduler/http/request.hpp"
#include "rmf2_scheduler/log.hpp"

MATCHER_P(contains_string_data, str, "") {
  if (arg->size() != str.size()) {
    return false;
  }

  std::string data;
  char buf[100];
  size_t read = 0;
  while (arg->read(buf, sizeof(buf), &read) && read > 0) {
    data.append(buf, read);
  }
  return data == str;
}

class TestHTTPRequest : public ::testing::Test
{
public:
  void SetUp() override
  {
    using testing::_;
    using testing::Return;
    using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)
    transport_ = std::make_shared<MockTransport>();
    connection_ = std::make_shared<MockConnection>();
  }

  void TearDown() override
  {
    testing::Mock::VerifyAndClearExpectations(connection_.get());
    connection_.reset();
    testing::Mock::VerifyAndClearExpectations(transport_.get());
    transport_.reset();
  }

protected:
  std::shared_ptr<rmf2_scheduler::http::MockTransport> transport_;
  std::shared_ptr<rmf2_scheduler::http::MockConnection> connection_;
};

TEST_F(TestHTTPRequest, defaults) {
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  {  // POST
    Request request{"http://www.foo.bar/post", request_type::kPost, transport_};
    EXPECT_TRUE(request.get_content_type().empty());
    EXPECT_TRUE(request.get_referer().empty());
    EXPECT_TRUE(request.get_user_agent().empty());
    EXPECT_EQ("*/*", request.get_accept());
    EXPECT_EQ("http://www.foo.bar/post", request.get_request_url());
    EXPECT_EQ(request_type::kPost, request.get_request_method());
  }

  {  // GET
    Request request{"http://www.foo.bar/get", request_type::kGet, transport_};
    EXPECT_EQ("http://www.foo.bar/get", request.get_request_url());
    EXPECT_EQ(request_type::kGet, request.get_request_method());
  }
}

TEST_F(TestHTTPRequest, content_type) {
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  Request request{"http://www.foo.bar", request_type::kPost, transport_};
  request.set_content_type("image/jpeg");
  EXPECT_EQ("image/jpeg", request.get_content_type());
}

TEST_F(TestHTTPRequest, referer) {
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  Request request{"http://www.foo.bar", request_type::kPost, transport_};
  request.set_referer("http://www.foo.bar/referer");
  EXPECT_EQ("http://www.foo.bar/referer", request.get_referer());
}

TEST_F(TestHTTPRequest, user_agent) {
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  Request request{"http://www.foo.bar", request_type::kPost, transport_};
  request.set_user_agent("FooBar Browser");
  EXPECT_EQ("FooBar Browser", request.get_user_agent());
}

TEST_F(TestHTTPRequest, accept) {
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  Request request{"http://www.foo.bar", request_type::kPost, transport_};
  request.set_accept("text/*, text/html, text/html;level=1, */*");
  EXPECT_EQ("text/*, text/html, text/html;level=1, */*", request.get_accept());
}

TEST_F(TestHTTPRequest, get_response_and_block) {
  using testing::_;
  using testing::Return;
  using testing::ByMove;
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  Request request{"http://www.foo.bar", request_type::kPost, transport_};
  request.set_user_agent("FooBar Browser");
  request.set_referer("http://www.foo.bar/baz");
  request.set_accept("text/*, text/html, text/html;level=1, */*");
  request.add_header(request_header::kAcceptEncoding, "compress, gzip");
  request.add_headers(
  {
    {request_header::kAcceptLanguage, "da, en-gb;q=0.8, en;q=0.7"},
    {request_header::kConnection, "close"},
  });
  request.add_range(-10);
  request.add_range(100, 200);
  request.add_range(300);
  std::string req_body{"Foo bar baz"};
  request.add_header(request_header::kContentType, "plain");

  std::string error;
  EXPECT_CALL(
    *transport_,
    create_connection(
      "http://www.foo.bar",
      request_type::kPost,
      HeaderList{
    {request_header::kAcceptEncoding, "compress, gzip"},
    {request_header::kAcceptLanguage, "da, en-gb;q=0.8, en;q=0.7"},
    {request_header::kConnection, "close"},
    {request_header::kContentType, "plain"},
    {request_header::kRange, "bytes=-10,100-200,300-"},
    {request_header::kAccept,
      "text/*, text/html, text/html;level=1, */*"},
  },
      "FooBar Browser", "http://www.foo.bar/baz", error
    )
  ).WillOnce(Return(connection_));

  EXPECT_CALL(
    *connection_,
    set_request_data(contains_string_data(req_body), _)
  ).WillOnce(Return(true));

  EXPECT_TRUE(
    request.add_request_body(
      req_body.data(),
      req_body.size(),
      error
    )
  );
  EXPECT_CALL(*connection_, perform_request(_)).WillOnce(Return(true));
  auto response = request.get_response_and_block(error);
  EXPECT_NE(nullptr, response.get());

  // Check response
  EXPECT_CALL(
    *connection_,
    get_response_status_code()
  ).WillRepeatedly(Return(status_code::Ok));

  EXPECT_CALL(
    *connection_,
    get_response_status_text()
  ).WillOnce(Return("OK"));

  EXPECT_CALL(
    *connection_,
    get_response_header(response_header::kContentType)
  ).WillOnce(Return("text/html"));

  std::string response_data = "<html><body>OK</body></html>";
  Stream::UPtr stream = MemoryStream::open_copy_of(response_data);
  EXPECT_CALL(
    *connection_,
    extract_data_stream()
  ).WillOnce(Return(ByMove(std::move(stream))));

  EXPECT_TRUE(response->is_successful());
  EXPECT_EQ(status_code::Ok, response->get_status_code());
  EXPECT_EQ("OK", response->get_status_text());
  EXPECT_EQ("text/html", response->get_content_type());
  EXPECT_EQ("<html><body>OK</body></html>", response->extract_data_as_string());
}
