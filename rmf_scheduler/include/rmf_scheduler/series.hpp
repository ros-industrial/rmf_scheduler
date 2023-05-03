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

#ifndef RMF_SCHEDULER__SERIES_HPP_
#define RMF_SCHEDULER__SERIES_HPP_

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rmf_scheduler/exception.hpp"

namespace cron
{

class cronexpr;

}  // namespace cron

namespace rmf_scheduler
{

class Series
{
public:
  struct Occurrence
  {
    uint64_t time;
    std::string id;
  };

  struct Description
  {
    std::vector<Occurrence> occurrences;
    std::string cron;
    uint64_t until;
    std::vector<std::string> exception_ids;
    std::string id_prefix;
  };

  Series();
  Series(const Series &);
  Series & operator=(const Series &);
  Series(Series &&);
  Series & operator=(Series && rhs);

  virtual ~Series();

  explicit Series(
    const Description & description);

  explicit Series(
    const std::string & id,
    uint64_t time,
    const std::string & cron,
    uint64_t until = UINT64_MAX,
    const std::string & id_prefix = "");

  explicit operator bool() const;

  Description description() const;

  std::string get_occurrence_id(uint64_t time) const;

  Occurrence get_last_occurrence() const;

  uint64_t get_until() const;

  std::string cron() const;

  std::vector<Occurrence> expand_until(uint64_t time);

  void update_occurrence_id(
    uint64_t time,
    const std::string & new_id);

  void update_cron_from(
    uint64_t time,
    uint64_t new_start_time,
    std::string new_cron);

  void update_occurrence_time(
    uint64_t time,
    uint64_t new_start_time);

  void set_id_prefix(
    const std::string & id_prefix);

  void set_until(
    uint64_t time);

  void delete_occurrence(
    uint64_t time);

private:
  bool _validate_time(uint64_t time, const std::unique_ptr<cron::cronexpr> &) const;

  std::unique_ptr<cron::cronexpr> cron_;

  uint64_t until_;

  std::string id_prefix_;

  std::map<uint64_t, std::string> occurrence_ids_;

  std::unordered_set<std::string> exception_ids_;
};


class SeriesEmptyException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  SeriesEmptyException(const char * msg, Args && ... args)
  : ExceptionTemplate(msg, std::forward<Args>(args) ...)
  {
    add_prefix("SeriesEmptyException: ");
  }
};

class SeriesInvalidCronException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  SeriesInvalidCronException(const char * msg, Args && ... args)
  : ExceptionTemplate(msg, std::forward<Args>(args) ...)
  {
    add_prefix("SeriesInvalidCronException: ");
  }
};

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__SERIES_HPP_
