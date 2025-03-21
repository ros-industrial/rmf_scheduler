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

#include "rmf2_scheduler/http/common.hpp"

namespace rmf2_scheduler
{

namespace http
{

// request_type
const char request_type::kOptions[] = "OPTIONS";
const char request_type::kGet[] = "GET";
const char request_type::kHead[] = "HEAD";
const char request_type::kPost[] = "POST";
const char request_type::kPut[] = "PUT";
const char request_type::kPatch[] = "PATCH";
const char request_type::kDelete[] = "DELETE";
const char request_type::kTrace[] = "TRACE";
const char request_type::kConnect[] = "CONNECT";
const char request_type::kCopy[] = "COPY";
const char request_type::kMove[] = "MOVE";

// request_header
const char request_header::kAccept[] = "Accept";
const char request_header::kAcceptCharset[] = "Accept-Charset";
const char request_header::kAcceptEncoding[] = "Accept-Encoding";
const char request_header::kAcceptLanguage[] = "Accept-Language";
const char request_header::kAllow[] = "Allow";
const char request_header::kAuthorization[] = "Authorization";
const char request_header::kCacheControl[] = "Cache-Control";
const char request_header::kConnection[] = "Connection";
const char request_header::kContentEncoding[] = "Content-Encoding";
const char request_header::kContentLanguage[] = "Content-Language";
const char request_header::kContentLength[] = "Content-Length";
const char request_header::kContentLocation[] = "Content-Location";
const char request_header::kContentMd5[] = "Content-MD5";
const char request_header::kContentRange[] = "Content-Range";
const char request_header::kContentType[] = "Content-Type";
const char request_header::kCookie[] = "Cookie";
const char request_header::kDate[] = "Date";
const char request_header::kExpect[] = "Expect";
const char request_header::kExpires[] = "Expires";
const char request_header::kFrom[] = "From";
const char request_header::kHost[] = "Host";
const char request_header::kIfMatch[] = "If-Match";
const char request_header::kIfModifiedSince[] = "If-Modified-Since";
const char request_header::kIfNoneMatch[] = "If-None-Match";
const char request_header::kIfRange[] = "If-Range";
const char request_header::kIfUnmodifiedSince[] = "If-Unmodified-Since";
const char request_header::kLastModified[] = "Last-Modified";
const char request_header::kMaxForwards[] = "Max-Forwards";
const char request_header::kPragma[] = "Pragma";
const char request_header::kProxyAuthorization[] = "Proxy-Authorization";
const char request_header::kRange[] = "Range";
const char request_header::kReferer[] = "Referer";
const char request_header::kTE[] = "TE";
const char request_header::kTrailer[] = "Trailer";
const char request_header::kTransferEncoding[] = "Transfer-Encoding";
const char request_header::kUpgrade[] = "Upgrade";
const char request_header::kUserAgent[] = "User-Agent";
const char request_header::kVia[] = "Via";
const char request_header::kWarning[] = "Warning";

// response_header
const char response_header::kAcceptRanges[] = "Accept-Ranges";
const char response_header::kAge[] = "Age";
const char response_header::kAllow[] = "Allow";
const char response_header::kCacheControl[] = "Cache-Control";
const char response_header::kConnection[] = "Connection";
const char response_header::kContentEncoding[] = "Content-Encoding";
const char response_header::kContentLanguage[] = "Content-Language";
const char response_header::kContentLength[] = "Content-Length";
const char response_header::kContentLocation[] = "Content-Location";
const char response_header::kContentMd5[] = "Content-MD5";
const char response_header::kContentRange[] = "Content-Range";
const char response_header::kContentType[] = "Content-Type";
const char response_header::kDate[] = "Date";
const char response_header::kETag[] = "ETag";
const char response_header::kExpires[] = "Expires";
const char response_header::kLastModified[] = "Last-Modified";
const char response_header::kLocation[] = "Location";
const char response_header::kPragma[] = "Pragma";
const char response_header::kProxyAuthenticate[] = "Proxy-Authenticate";
const char response_header::kRetryAfter[] = "Retry-After";
const char response_header::kServer[] = "Server";
const char response_header::kSetCookie[] = "Set-Cookie";
const char response_header::kTrailer[] = "Trailer";
const char response_header::kTransferEncoding[] = "Transfer-Encoding";
const char response_header::kUpgrade[] = "Upgrade";
const char response_header::kVary[] = "Vary";
const char response_header::kVia[] = "Via";
const char response_header::kWarning[] = "Warning";
const char response_header::kWwwAuthenticate[] = "WWW-Authenticate";

}  // namespace http
}  // namespace rmf2_scheduler
