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

#include "rmf2_scheduler/http/mock_curl_api.hpp"
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
  using HeaderList = std::vector<std::pair<std::string, std::string>>;

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
  ReadWriteCallback * write_callback{nullptr};
  ReadWriteCallback * read_callback{nullptr};
  ReadWriteCallback * header_callback{nullptr};

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
      if (size_written <= size_remaining) {
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

class TestHTTPConnection : public ::testing::Test
{
};

TEST_F(TestHTTPConnection, empty) {
  rmf2_scheduler::log::setLogLevel(rmf2_scheduler::LogLevel::DEBUG);
  std::shared_ptr<rmf2_scheduler::http::Connection> connection;
  std::shared_ptr<rmf2_scheduler::http::CURLInterface> curl_interface;
  curl_interface = std::make_shared<rmf2_scheduler::http::MockCURLInterface>();
  CURL * eh = curl_interface->easy_init();
  curl_interface->easy_setopt_str(
    eh, CURLOPT_URL,
    "http://localhost:9090/ngsi-ld/v1/entities?"
    "type=urn:rmf2:event&q=startTime%3E=2021-01-15T14:45:00Z"
  );
  connection = std::make_shared<rmf2_scheduler::http::curl::Connection>(
    eh,
    "GET",
    curl_interface
  );

  std::string error;
  EXPECT_TRUE(connection->perform_request(error));
  std::cout << error << std::endl;
  std::cout << connection->extract_data_stream() << std::endl;
}
