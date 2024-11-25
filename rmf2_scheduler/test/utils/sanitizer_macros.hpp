// Copyright 2023-2024 ROS Industrial Consortium Asia Pacific
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

#ifndef UTILS__SANITIZER_MACROS_HPP_
#define UTILS__SANITIZER_MACROS_HPP_

// Align sanitizer in Clang with GCC
#ifdef __has_feature
#if __has_feature(address_sanitizer)
// code that builds only under addresssanitizer
// only works in clang
// so define the __sanitize_address__ to align with gcc
#define __SANITIZE_ADDRESS__
#endif
#if __has_feature(thread_sanitizer)
// similarly for thread sanitizer
#define __SANITIZE_THREAD__
#endif
#endif

#endif  // UTILS__SANITIZER_MACROS_HPP_
