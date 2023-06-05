// Copyright 2023 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF_SCHEDULER__EXCEPTION_HPP_
#define RMF_SCHEDULER__EXCEPTION_HPP_

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <exception>
#include <utility>

namespace rmf_scheduler
{

class ExceptionTemplate : public std::exception
{
public:
  ExceptionTemplate(
    const char * msg, ...)
  {
    buffer_ = new char[4096];
    va_list args;
    va_start(args, msg);
    vsprintf(buffer_, msg, args);
    va_end(args);
  }

  virtual ~ExceptionTemplate()
  {
    delete[] buffer_;
  }

  const char * what() const noexcept
  {
    return buffer_;
  }

protected:
  void add_prefix(const char * prefix)
  {
    char * new_buffer;
    size_t new_buffer_len = strlen(buffer_) + strlen(prefix) + 1;
    new_buffer = new char[new_buffer_len];
    snprintf(new_buffer, new_buffer_len, "%s%s", prefix, buffer_);
    delete[] buffer_;
    buffer_ = new_buffer;
  }

private:
  char * buffer_;
};

class IDException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  IDException(const char * id, const char * msg, Args && ... args)
  : ExceptionTemplate(msg, std::forward<Args>(args) ...)
  {
    // Copy over id
    size_t id_len = strlen(id) + 1;
    id_ = new char[id_len];
    snprintf(id_, id_len, "%s", id);
  }

  virtual ~IDException()
  {
    delete[] id_;
  }

  const char * id() const
  {
    return id_;
  }

private:
  char * id_;
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__EXCEPTION_HPP_
