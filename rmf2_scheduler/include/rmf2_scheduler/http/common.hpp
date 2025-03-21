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

#ifndef RMF2_SCHEDULER__HTTP__COMMON_HPP_
#define RMF2_SCHEDULER__HTTP__COMMON_HPP_

#include <string>
#include <utility>
#include <vector>

namespace rmf2_scheduler
{

namespace http
{

using HeaderList = std::vector<std::pair<std::string, std::string>>;

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

}  // namespace http

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__HTTP__COMMON_HPP_
