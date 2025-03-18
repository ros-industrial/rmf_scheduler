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

#ifndef RMF2_SCHEDULER__HTTP__REQUEST_HPP_
#define RMF2_SCHEDULER__HTTP__REQUEST_HPP_

#include <map>
#include <memory>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "rmf2_scheduler/http/transport.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace rmf2_scheduler
{

namespace http
{

class Response;

// HTTP request verbs
namespace request_type
{

extern const char kOptions[];
extern const char kGet[];
extern const char kHead[];
extern const char kPost[];
extern const char kPut[];
extern const char kPatch[];  // Non-standard HTTP/1.1 verb
extern const char kDelete[];
extern const char kTrace[];
extern const char kConnect[];
extern const char kCopy[];   // Non-standard HTTP/1.1 verb
extern const char kMove[];   // Non-standard HTTP/1.1 verb

}  // namespace request_type

// HTTP request header names
namespace request_header
{
extern const char kAccept[];
extern const char kAcceptCharset[];
extern const char kAcceptEncoding[];
extern const char kAcceptLanguage[];
extern const char kAllow[];
extern const char kAuthorization[];
extern const char kCacheControl[];
extern const char kConnection[];
extern const char kContentEncoding[];
extern const char kContentLanguage[];
extern const char kContentLength[];
extern const char kContentLocation[];
extern const char kContentMd5[];
extern const char kContentRange[];
extern const char kContentType[];
extern const char kCookie[];
extern const char kDate[];
extern const char kExpect[];
extern const char kExpires[];
extern const char kFrom[];
extern const char kHost[];
extern const char kIfMatch[];
extern const char kIfModifiedSince[];
extern const char kIfNoneMatch[];
extern const char kIfRange[];
extern const char kIfUnmodifiedSince[];
extern const char kLastModified[];
extern const char kMaxForwards[];
extern const char kPragma[];
extern const char kProxyAuthorization[];
extern const char kRange[];
extern const char kReferer[];
extern const char kTE[];
extern const char kTrailer[];
extern const char kTransferEncoding[];
extern const char kUpgrade[];
extern const char kUserAgent[];
extern const char kVia[];
extern const char kWarning[];
}  // namespace request_header

// HTTP response header names
namespace response_header
{
extern const char kAcceptRanges[];
extern const char kAge[];
extern const char kAllow[];
extern const char kCacheControl[];
extern const char kConnection[];
extern const char kContentEncoding[];
extern const char kContentLanguage[];
extern const char kContentLength[];
extern const char kContentLocation[];
extern const char kContentMd5[];
extern const char kContentRange[];
extern const char kContentType[];
extern const char kDate[];
extern const char kETag[];
extern const char kExpires[];
extern const char kLastModified[];
extern const char kLocation[];
extern const char kPragma[];
extern const char kProxyAuthenticate[];
extern const char kRetryAfter[];
extern const char kServer[];
extern const char kSetCookie[];
extern const char kTrailer[];
extern const char kTransferEncoding[];
extern const char kUpgrade[];
extern const char kVary[];
extern const char kVia[];
extern const char kWarning[];
extern const char kWwwAuthenticate[];
}  // namespace response_header

// HTTP request status (error) codes
namespace status_code
{
// OK to continue with request
static const int Continue = 100;
// Server has switched protocols in upgrade header
static const int SwitchProtocols = 101;
// Request completed
static const int Ok = 200;
// Object created, reason = new URI
static const int Created = 201;
// Async completion (TBS)
static const int Accepted = 202;
// Partial completion
static const int Partial = 203;
// No info to return
static const int NoContent = 204;
// Request completed, but clear form
static const int ResetContent = 205;
// Partial GET fulfilled
static const int PartialContent = 206;
// Server couldn't decide what to return
static const int Ambiguous = 300;
// Object permanently moved
static const int Moved = 301;
// Object temporarily moved
static const int Redirect = 302;
// Redirection w/ new access method
static const int RedirectMethod = 303;
// If-Modified-Since was not modified
static const int NotModified = 304;
// Redirection to proxy, location header specifies proxy to use
static const int UseProxy = 305;
// HTTP/1.1: keep same verb
static const int RedirectKeepVerb = 307;
// Invalid syntax
static const int BadRequest = 400;
// Access denied
static const int Denied = 401;
// Payment required
static const int PaymentRequired = 402;
// Request forbidden
static const int Forbidden = 403;
// Object not found
static const int NotFound = 404;
// Method is not allowed
static const int BadMethod = 405;
// No response acceptable to client found
static const int NoneAcceptable = 406;
// Proxy authentication required
static const int ProxyAuthRequired = 407;
// Server timed out waiting for request
static const int RequestTimeout = 408;
// User should resubmit with more info
static const int Conflict = 409;
// The resource is no longer available
static const int Gone = 410;
// The server refused to accept request w/o a length
static const int LengthRequired = 411;
// Precondition given in request failed
static const int PrecondionFailed = 412;
// Request entity was too large
static const int RequestTooLarge = 413;
// Request URI too long
static const int UriTooLong = 414;
// Unsupported media type
static const int UnsupportedMedia = 415;
// Too many requests
static const int TooManyRequests = 429;
// Retry after doing the appropriate action.
static const int RetryWith = 449;
// Internal server error
static const int InternalServerError = 500;
// Request not supported
static const int NotSupported = 501;
// Error response received from gateway
static const int BadGateway = 502;
// Temporarily overloaded
static const int ServiceUnavailable = 503;
// Timed out waiting for gateway
static const int GatewayTimeout = 504;
// HTTP version not supported
static const int VersionNotSupported = 505;
}  // namespace status_code

class Request final
{
public:
  using HeaderList = std::vector<std::pair<std::string, std::string>>;

  // The main constructor. |url| specifies the remote host address/path
  // to send the request to. |method| is the HTTP request verb and
  // |transport| is the HTTP transport implementation for server communications.
  Request(
    const std::string & url,
    const std::string & method,
    std::shared_ptr<Transport> transport
  );

  ~Request();

  // Gets/Sets "Accept:" header value. The default value is "*/*" if not set.
  void set_accept(const std::string & accept_mime_types);
  const std::string & get_accept() const;

  // Gets/Sets "Content-Type:" header value
  void set_content_type(const std::string & content_type);
  const std::string & get_content_type() const;

  // Adds additional HTTP request header
  void add_header(const std::string & header, const std::string & value);
  void add_headers(const HeaderList & headers);

  // Removes HTTP request header
  void remove_header(const std::string & header);

  // Adds a request body. This is not to be used with GET method
  // bool add_request_body(const void * data, size_t size, std::string & error);
  bool add_request_body(const std::string & stream, std::string & error);

  // // Adds a request body. This is not to be used with GET method.
  // // This method also sets the correct content-type of the request, including
  // // the multipart data boundary.
  // bool AddRequestBodyAsFormData(std::unique_ptr<FormData> form_data,
  //                               brillo::ErrorPtr* error);

  // // Adds a stream for the response. Otherwise a MemoryStream will be used.
  // bool AddResponseStream(StreamPtr stream, brillo::ErrorPtr* error);

  // // Makes a request for a subrange of data. Specifies a partial range with
  // // either from beginning of the data to the specified offset (if |bytes| is
  // // negative) or from the specified offset to the end of data (if |bytes| is
  // // positive).
  // // All individual ranges will be sent as part of "Range:" HTTP request header.
  // void AddRange(int64_t bytes);
  // // Makes a request for a subrange of data. Specifies a full range with
  // // start and end bytes from the beginning of the requested data.
  // // All individual ranges will be sent as part of "Range:" HTTP request header.
  // void AddRange(uint64_t from_byte, uint64_t to_byte);

  // Returns the request URL
  const std::string & get_request_url() const;

  // Returns the request verb.
  const std::string & get_request_method() const;

  // Gets/Sets a request referer URL (sent as "Referer:" request header).
  void set_referer(const std::string & referer);
  const std::string & get_referer() const;

  // Gets/Sets a user agent string (sent as "User-Agent:" request header).
  void set_user_agent(const std::string & user_agent);
  const std::string & get_user_agent() const;

  // Sends the request to the server and blocks until the response is received,
  // which is returned as the response object.
  // In case the server couldn't be reached for whatever reason, returns
  // empty unique_ptr (null). In such a case, the additional error information
  // can be returned through the optional supplied |error| parameter.
  std::unique_ptr<Response> get_response_and_block(std::string & error);

private:
  // Helper function to create an http::Connection and send off request headers.
  bool _send_request_if_needed(std::string & error);

  // Implementation that provides particular HTTP transport.
  std::shared_ptr<Transport> transport_;
  // An established connection for adding request body. This connection
  // is maintained by the request object after the headers have been
  // sent and before the response is requested.
  std::shared_ptr<Connection> connection_;
  // Full request URL, such as "http://www.host.com/path/to/object"
  const std::string request_url_;
  // HTTP request verb, such as "GET", "POST", "PUT", ...
  const std::string method_;
  // Referrer URL, if any. Sent to the server via "Referer: " header.
  std::string referer_;
  // User agent string, if any. Sent to the server via "User-Agent: " header.
  std::string user_agent_;
  // Content type of the request body data.
  // Sent to the server via "Content-Type: " header.
  std::string content_type_;
  // List of acceptable response data types.
  // Sent to the server via "Accept: " header.
  std::string accept_ = "*/*";
  // List of optional request headers provided by the caller.
  std::multimap<std::string, std::string> headers_;

  // // List of optional data ranges to request partial content from the server.
  // // Sent to the server as "Range: " header.
  // std::vector<std::pair<uint64_t, uint64_t>> ranges_;
  // // range_value_omitted is used in |ranges_| list to indicate omitted value.
  // // E.g. range (10,range_value_omitted) represents bytes from 10 to the end
  // // of the data stream.
  // const uint64_t range_value_omitted = std::numeric_limits<uint64_t>::max();

private:
  RS_DISABLE_COPY(Request)
};

class Response final
{
public:
  explicit Response(const std::shared_ptr<Connection> & connection);
  ~Response();

  // Returns true if server returned a success code (status code below 400).
  bool is_successful() const;

  // Returns the HTTP status code (e.g. 200 for success)
  int get_status_code() const;

  // Returns the status text (e.g. for error 403 it could be "NOT AUTHORIZED").
  std::string get_status_text() const;

  // Returns the content type of the response data.
  std::string get_content_type() const;

  // Returns response data stream by transferring ownership of the data stream
  // from Response class to the caller.
  std::string extract_data_stream();

  // // Extracts the data from the underlying response data stream as a byte array.
  // std::vector<uint8_t> extract_data();

  // // Extracts the data from the underlying response data stream as a string.
  // std::string extract_data_as_string();

  // Returns a value of a given response HTTP header.
  std::string get_header(const std::string & header_name) const;

private:
  RS_DISABLE_COPY(Response)

  std::shared_ptr<Connection> connection_;
};

}  // namespace http

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__REQUEST_HPP_
