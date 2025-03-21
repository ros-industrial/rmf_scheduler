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

#ifndef RMF2_SCHEDULER__HTTP__TRANSPORT_FAKE_HPP_
#define RMF2_SCHEDULER__HTTP__TRANSPORT_FAKE_HPP_

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "nlohmann/json.hpp"
#include "rmf2_scheduler/http/transport.hpp"

namespace rmf2_scheduler
{

namespace http
{

namespace fake
{

class ServerRequest;
class ServerResponse;
class Connection;

// Server handler callback signature.
using HandlerCallback =
  std::function<void (const ServerRequest &, ServerResponse *)>;

/// A fake implementation of http::Transport that simulates HTTP communication
/// with a server.
class Transport : public http::Transport
{
public:
  Transport();

  virtual ~Transport();

  /// Add a handler
  /**
   * This method allows the test code to provide a callback to handle requests
   * for specific URL/HTTP-verb combination. When a specific |method| request
   * is made on the given |url|, the |handler| will be invoked and all the
   * request data will be filled in the |ServerRequest| parameter. Any server
   * response should be returned through the |ServerResponse| parameter.
   * Either |method| or |url| (or both) can be specified as "*" to handle
   * any requests. So, ("http://localhost","*") will handle any request type
   * on that URL and ("*","GET") will handle any GET requests.
   * The lookup starts with the most specific data pair to the catch-all (*,*).
   */
  void add_handler(
    const std::string & url,
    const std::string & method,
    const HandlerCallback & handler
  );

  /// Add a simple version of the handler
  /**
   * This method just returns the specified text response of given MIME type.
   */
  void add_simple_reply_handler(
    const std::string & url,
    const std::string & method,
    int status_code,
    const std::string & reply_text,
    const std::string & mime_type
  );

  /// Retrieve a handler for specific |url| and request |method|.
  HandlerCallback get_handler(
    const std::string & url,
    const std::string & method
  ) const;

  /// Check request counts
  /**
   * For tests that want to assert on the number of HTTP requests sent,
   * these methods can be used to do just that.
   */
  int get_request_count() const
  {
    return request_count_;
  }

  void reset_request_count()
  {
    request_count_ = 0;
  }

  // For tests that wish to simulate critical transport errors, this method
  // can be used to specify the error to be returned when creating a connection.
  void set_create_connection_error(const std::string & create_connection_error)
  {
    create_connection_error_ = create_connection_error;
  }

  std::shared_ptr<http::Connection> create_connection(
    const std::string & url,
    const std::string & method,
    const HeaderList & headers,
    const std::string & user_agent,
    const std::string & referer,
    std::string & error
  ) override;

private:
  RS_DISABLE_COPY(Transport)

  // A list of user-supplied request handlers.
  std::unordered_map<std::string, HandlerCallback> handlers_;

  int request_count_ = 0;

  std::string create_connection_error_;
};

class ServerRequestResponseBase
{
public:
  ServerRequestResponseBase();
  virtual ~ServerRequestResponseBase();

  /// Add/Retrieve request/respones body data.
  void set_data(Stream::UPtr data);
  const std::vector<uint8_t> & get_data() const {return data_;}

  std::string get_data_as_string() const;
  std::optional<nlohmann::json> get_data_as_json() const;
  std::string get_data_as_normalized_json_string() const;

  /// Add/Retrieve request/response HTTP headers.
  void add_headers(const HeaderList & headers);
  std::string get_header(const std::string & header_name) const;
  const std::multimap<std::string, std::string> & get_headers() const
  {
    return headers_;
  }

protected:
  RS_DISABLE_COPY(ServerRequestResponseBase)

  // Data buffer.
  std::vector<uint8_t> data_;
  // Header map.
  std::multimap<std::string, std::string> headers_;
};


class ServerRequest : public ServerRequestResponseBase
{
public:
  explicit ServerRequest(const std::string & url, const std::string & method);

  virtual ~ServerRequest();

  /// Get the actual request URL. Does not include the query string or fragment.
  const std::string & get_url() const {return url_;}

  /// Get the request method.
  const std::string & get_method() const {return method_;}

  /// Get the POST/GET request parameters.
  /**
   * These are parsed query string
   * parameters from the URL. In addition, for POST requests with
   * application/x-www-form-urlencoded content type, the request body is also
   * parsed and individual fields can be accessed through this method.
   */
  std::string get_form_field(const std::string & field_name) const;

private:
  RS_DISABLE_COPY(ServerRequest)

  // Request URL (without query string or URL fragment).
  std::string url_;

  // Request method
  std::string method_;

  // List of available request data form fields.
  mutable std::map<std::string, std::string> form_fields_;

  // Flag used on first request to GetFormField to parse the body of HTTP POST
  // request with application/x-www-form-urlencoded content.
  mutable bool form_fields_parsed_ = false;
};

class ServerResponse : public ServerRequestResponseBase
{
public:
  ServerResponse();
  virtual ~ServerResponse();

  /// Generic reply method.
  void reply(
    int status_code,
    const void * data,
    size_t data_size,
    const std::string & mime_type
  );

  /// Reply with text body.
  void reply_text(
    int status_code,
    const std::string & text,
    const std::string & mime_type
  );

  /// Reply with JSON object. The content type will be "application/json".
  void reply_json(
    int status_code,
    const nlohmann::json & json,
    const std::string & mime_type = "application/json"
  );

  /// Reply with binary data (array)
  /**
   * Specialized overload to send the binary data as an array of simple
   * data elements. Only trivial data types (scalars, POD structures, etc)
   * can be used.
   */
  template<typename T>
  void reply(
    int status_code,
    const std::vector<T> & data,
    const std::string & mime_type
  )
  {
    // Make sure T doesn't have virtual functions, custom constructors, etc.
    static_assert(std::is_trivial<T>::value, "Only simple data is supported");
    reply(status_code, data.data(), data.size() * sizeof(T), mime_type);
  }

  /// Reply with binary data (blob)
  /**
   * Specialized overload to send the binary data.
   * Only trivial data types (scalars, POD structures, etc) can be used.
   */
  template<typename T>
  void reply(
    int status_code,
    const T & data,
    const std::string & mime_type
  )
  {
    // Make sure T doesn't have virtual functions, custom constructors, etc.
    static_assert(std::is_trivial<T>::value, "Only simple data is supported");
    reply(status_code, &data, sizeof(T), mime_type);
  }

  /// Set HTTP protocol version
  /**
   * For handlers that want to simulate versions of HTTP protocol other
   * than HTTP/1.1, call this method with the custom version string,
   * for example "HTTP/1.0".
   */
  void set_protocol_version(const std::string & protocol_version)
  {
    protocol_version_ = protocol_version;
  }

protected:
  // These methods are helpers to implement corresponding functionality
  // of fake::Connection.

  friend class Connection;

  /// Helper for fake::Connection::get_response_status_code().
  int get_status_code() const {return status_code_;}

  /// Helper for fake::Connection::get_response_status_text().
  std::string get_status_text() const;

  // Helper for fake::Connection::get_protocol_version().
  std::string get_protocol_version() const {return protocol_version_;}

private:
  RS_DISABLE_COPY(ServerResponse)
  int status_code_ = 0;
  std::string protocol_version_ = "HTTP/1.1";
};

class ServerRequestHandler
{
public:
  ServerRequestHandler() {}
  virtual ~ServerRequestHandler() {}

  void add_callback(
    HandlerCallback handler_callback
  )
  {
    handler_callbacks_.push_back(handler_callback);
  }

  void perform(
    const ServerRequest & request, ServerResponse * response
  );

  size_t call_count() const
  {
    return call_count_;
  }

private:
  RS_DISABLE_COPY(ServerRequestHandler)
  std::vector<HandlerCallback> handler_callbacks_;
  size_t call_count_ = 0;
};

}  // namespace fake
}  // namespace http
}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__TRANSPORT_FAKE_HPP_
