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

#include "rmf2_scheduler/utils/string_utils.hpp"
#include "rmf2_scheduler/http/mock_curl_api.hpp"
#include "rmf2_scheduler/http/memory_stream.hpp"
#include "rmf2_scheduler/http/mock_stream.hpp"
#include "rmf2_scheduler/http/request.hpp"
#include "rmf2_scheduler/http/connection_curl.hpp"
#include "rmf2_scheduler/log.hpp"

namespace rmf2_scheduler
{

namespace http
{

namespace curl
{

// A helper class to simulate curl_easy_perform action. It invokes the
// read callbacks to obtain the request data from the Connection and then
// calls the header and write callbacks to "send" the response header and body.
class CURLPerformer
{
public:
  using ReadWriteCallback = size_t (char * ptr, size_t size, size_t num, void * data);

  // During the tests, use the address of |this| as the CURL* handle.
  // This allows the static Perform() method to obtain the instance pointer
  // having only CURL*.
  CURL * get_curl_handle() {return reinterpret_cast<CURL *>(this);}

  // Callback to be invoked when mocking out curl_easy_perform() method.
  static CURLcode perform(CURL * curl)
  {
    CURLPerformer * me = reinterpret_cast<CURLPerformer *>(curl);
    return me->_do_perform();
  }

  // CURL callback functions and |connection| pointer needed to invoke the
  // callbacks from the Connection class.
  Connection * connection = nullptr;
  ReadWriteCallback * write_callback = nullptr;
  ReadWriteCallback * read_callback = nullptr;
  ReadWriteCallback * header_callback = nullptr;

  // Request body read from the connection.
  std::string request_body;

  // Response data to be sent back to connection.
  std::string status_line;
  HeaderList response_headers;
  std::string response_body;

private:
  // The actual implementation of curl_easy_perform() fake.
  CURLcode _do_perform()
  {
    // Read request body.
    char buffer[1024];
    for (;; ) {
      size_t size_read = read_callback(buffer, sizeof(buffer), 1, connection);
      if (size_read == CURL_READFUNC_ABORT) {
        return CURLE_ABORTED_BY_CALLBACK;
      }
      if (size_read == CURL_READFUNC_PAUSE) {
        return CURLE_READ_ERROR;  // Shouldn't happen.
      }
      if (size_read == 0) {
        break;
      }
      request_body.append(buffer, size_read);
    }
    // Send the response headers.
    std::vector<std::string> header_lines;
    header_lines.push_back(status_line + "\r\n");
    for (const auto & pair : response_headers) {
      std::ostringstream oss;
      oss << pair.first << ": " << pair.second << "\r\n";
      header_lines.push_back(oss.str());
    }
    for (const std::string & line : header_lines) {
      CURLcode code = _write_string(header_callback, line);
      if (code != CURLE_OK) {
        return code;
      }
    }
    // Send response body.
    return _write_string(write_callback, response_body);
  }

  // Helper method to send a string to a write callback. Keeps calling
  // the callback until all the data is written.
  CURLcode _write_string(ReadWriteCallback * callback, const std::string & str)
  {
    size_t pos = 0;
    size_t size_remaining = str.size();
    while (size_remaining) {
      size_t size_written = callback(
        const_cast<char *>(str.data() + pos),
        size_remaining, 1, connection);
      if (size_written == CURL_WRITEFUNC_PAUSE) {
        return CURLE_WRITE_ERROR;  // Shouldn't happen.
      }
      if (size_written > size_remaining) {
        throw std::runtime_error("Unexpected size returned");
      }
      size_remaining -= size_written;
      pos += size_written;
    }
    return CURLE_OK;
  }
};

}  // namespace curl
}  // namespace http
}  // namespace rmf2_scheduler

// Custom matcher to validate the parameter of CURLOPT_HTTPHEADER CURL option
// which contains the request headers as curl_slist* chain.
MATCHER_P(headers_match, headers, "") {
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)

  std::multiset<std::string> test_headers;
  for (const auto & pair : headers) {
    test_headers.insert(string_utils::join(": ", pair.first, pair.second));
  }
  std::multiset<std::string> src_headers;
  const curl_slist * head = static_cast<const curl_slist *>(arg);
  while (head) {
    src_headers.insert(head->data);
    head = head->next;
  }
  std::vector<std::string> difference;
  std::set_symmetric_difference(
    src_headers.begin(), src_headers.end(),
    test_headers.begin(), test_headers.end(),
    std::back_inserter(difference));
  return difference.empty();
}

// Custom action to save a CURL callback pointer into a member of CURLPerformer
ACTION_TEMPLATE(
  save_callback,
  HAS_1_TEMPLATE_PARAMS(int, k),
  AND_2_VALUE_PARAMS(performer, mem_ptr)) {
  using namespace rmf2_scheduler::http::curl;  // NOLINT(build/namespaces)
  performer->*mem_ptr = reinterpret_cast<CURLPerformer::ReadWriteCallback *>(std::get<k>(args));
}

class TestHTTPConnectionCURL : public ::testing::Test
{
public:
  void SetUp() override
  {
    using testing::_;
    using testing::Return;
    using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

    handle_ = performer_.get_curl_handle();
    curl_api_ = std::make_shared<MockCURLInterface>();
    EXPECT_CALL(
      *curl_api_,
      easy_setopt_ptr(handle_, CURLOPT_PRIVATE, _)
    ).WillOnce(Return(CURLE_OK));
    connection_ = std::make_shared<curl::Connection>(
      handle_, request_type::kPost, curl_api_
    );

    performer_.connection = connection_.get();
  }

  void TearDown() override
  {
    EXPECT_CALL(*curl_api_, easy_cleanup(handle_)).Times(1);
    connection_.reset();
    curl_api_.reset();
  }

protected:
  std::shared_ptr<rmf2_scheduler::http::MockCURLInterface> curl_api_;

  rmf2_scheduler::http::curl::CURLPerformer performer_;
  CURL * handle_ = nullptr;
  std::shared_ptr<rmf2_scheduler::http::curl::Connection> connection_;
};

MATCHER_P(match_string_buffer, data, "") {
  return data.compare(static_cast<const char *>(arg)) == 0;
}

TEST_F(TestHTTPConnectionCURL, perform_request) {
  using testing::_;
  using testing::DoAll;
  using testing::Invoke;
  using testing::Return;
  using testing::SetArgPointee;
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  std::string request_data = "Foo Bar Baz";
  std::string response_data = "<html><body>OK</body></html>";

  HeaderList headers{
    {request_header::kAccept, "*/*"},
    {request_header::kContentType, "plain"},
    {request_header::kContentLength, std::to_string(request_data.size())},
    {"X-Foo", "bar"},
  };

  // Create request and response stream
  Stream::UPtr request_stream = MemoryStream::open_copy_of(request_data);
  std::unique_ptr<MockStream> response_stream = std::make_unique<MockStream>();

  // Check response buffer
  EXPECT_CALL(
    *response_stream,
    write(match_string_buffer(response_data), response_data.size(), _)
  ).WillOnce(
    SetArgPointee<2>(response_data.size())
  );
  EXPECT_CALL(
    *response_stream,
    set_position(0)
  ).Times(1);

  std::string error;
  connection_->set_response_data(std::move(response_stream));
  EXPECT_TRUE(connection_->set_request_data(std::move(request_stream), error));
  EXPECT_TRUE(error.empty());
  EXPECT_TRUE(connection_->send_headers(headers, error));
  EXPECT_TRUE(error.empty());

  // Check CURL verbose configuration is called
  if (log::getLogLevel() <= LogLevel::DEBUG) {
    EXPECT_CALL(
      *curl_api_,
      easy_setopt_callback(handle_, CURLOPT_DEBUGFUNCTION, _)
    ).WillOnce(Return(CURLE_OK));
    EXPECT_CALL(
      *curl_api_,
      easy_setopt_int(handle_, CURLOPT_DEBUGFUNCTION, 1)
    ).WillOnce(Return(CURLE_OK));
  }

  // Check setting CURL message size
  EXPECT_CALL(
    *curl_api_,
    easy_setopt_off_t(handle_, CURLOPT_POSTFIELDSIZE_LARGE, request_data.size())
  ).WillOnce(Return(CURLE_OK));

  // Check setting CURL read callback and read data,
  // and move the read_callback to the performer
  EXPECT_CALL(
    *curl_api_,
    easy_setopt_callback(handle_, CURLOPT_READFUNCTION, _)
  ).WillOnce(
    DoAll(
      save_callback<2>(&performer_, &curl::CURLPerformer::read_callback),
      Return(CURLE_OK)
    )
  );
  EXPECT_CALL(
    *curl_api_,
    easy_setopt_ptr(handle_, CURLOPT_READDATA, _)
  ).WillOnce(Return(CURLE_OK));

  // Check header
  EXPECT_CALL(
    *curl_api_,
    easy_setopt_ptr(handle_, CURLOPT_HTTPHEADER, headers_match(headers))
  ).WillOnce(Return(CURLE_OK));

  // Check setting CURL write callback and write data,
  // and move the write_callback to the performer
  EXPECT_CALL(
    *curl_api_,
    easy_setopt_callback(handle_, CURLOPT_WRITEFUNCTION, _)
  ).WillOnce(
    DoAll(
      save_callback<2>(&performer_, &curl::CURLPerformer::write_callback),
      Return(CURLE_OK)
    )
  );
  EXPECT_CALL(
    *curl_api_,
    easy_setopt_ptr(handle_, CURLOPT_WRITEDATA, _)
  ).WillOnce(Return(CURLE_OK));

  // Check setting CURL header callback and header data,
  // and move the header_callback to the performer
  EXPECT_CALL(
    *curl_api_,
    easy_setopt_callback(handle_, CURLOPT_HEADERFUNCTION, _)
  ).WillOnce(
    DoAll(
      save_callback<2>(&performer_, &curl::CURLPerformer::header_callback),
      Return(CURLE_OK)
    )
  );
  EXPECT_CALL(
    *curl_api_,
    easy_setopt_ptr(handle_, CURLOPT_HEADERDATA, _)
  ).WillOnce(Return(CURLE_OK));

  // Override easy perform with CURLPerformer perform
  EXPECT_CALL(*curl_api_, easy_perform(handle_)).WillOnce(
    Invoke(&curl::CURLPerformer::perform)
  );

  // Override status code retrieval
  EXPECT_CALL(
    *curl_api_,
    easy_getinfo_int(handle_, CURLINFO_RESPONSE_CODE, _)
  ).WillOnce(
    DoAll(
      SetArgPointee<2>(status_code::Ok),
      Return(CURLE_OK)
    )
  );

  // Set up the CurlPerformer with the response data expected to be received.
  HeaderList response_headers{
    {response_header::kContentLength, std::to_string(response_data.size())},
    {response_header::kContentType, "text/html"},
    {"X-Foo", "baz"},
    {"Case-Insensitive", "response"},
    {"Case-Sensitive-Content", "Case Sensitive Content"},
  };
  performer_.status_line = "HTTP/1.1 200 OK";
  performer_.response_body = response_data;
  performer_.response_headers = response_headers;

  // Perform the request
  EXPECT_TRUE(connection_->perform_request(error));
  EXPECT_TRUE(error.empty());

  // Make sure we sent out the request body correctly.
  EXPECT_EQ(request_data, performer_.request_body);

  // Validate the parsed response data.
  EXPECT_CALL(
    *curl_api_,
    easy_getinfo_int(handle_, CURLINFO_RESPONSE_CODE, _)
  ).WillOnce(
    DoAll(
      SetArgPointee<2>(status_code::Ok),
      Return(CURLE_OK)
    )
  );
  EXPECT_EQ(status_code::Ok, connection_->get_response_status_code());
  EXPECT_EQ("HTTP/1.1", connection_->get_protocol_version());
  EXPECT_EQ("OK", connection_->get_response_status_text());
  EXPECT_EQ(
    std::to_string(response_data.size()),
    connection_->get_response_header(response_header::kContentLength)
  );
  EXPECT_EQ(
    "text/html",
    connection_->get_response_header(response_header::kContentType)
  );
  EXPECT_EQ("baz", connection_->get_response_header("X-Foo"));
  EXPECT_EQ("response", connection_->get_response_header("casE-insensitivE"));
  EXPECT_EQ(
    "Case Sensitive Content",
    connection_->get_response_header("case-sensitive-content")
  );
  auto data_stream = connection_->extract_data_stream();
  ASSERT_TRUE(data_stream);
}
