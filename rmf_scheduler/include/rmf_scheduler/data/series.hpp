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

#ifndef RMF_SCHEDULER__DATA__SERIES_HPP_
#define RMF_SCHEDULER__DATA__SERIES_HPP_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rmf_scheduler/exception.hpp"
#include "rmf_scheduler/error_code.hpp"

namespace cron
{

class cronexpr;

}  // namespace cron

namespace rmf_scheduler
{

namespace data
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
    std::string timezone;
    uint64_t until;
    std::vector<std::string> exception_ids;
    std::string id_prefix;
  };

  class ObserverBase
  {
public:
    virtual ~ObserverBase() {}
    virtual void on_add_occurrence(
      const Occurrence & ref_occurrence_id,
      const Occurrence & new_occurrence) = 0;

    virtual void on_update_occurrence(
      const Occurrence & old_occurrence,
      uint64_t new_time) = 0;

    virtual void on_delete_occurrence(
      const Occurrence & occurrence) = 0;
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
    const std::string & timezone,
    uint64_t until = UINT64_MAX,
    const std::string & id_prefix = "");

  explicit operator bool() const;

  Description description() const;

  std::vector<Occurrence> occurrences() const;

  std::string get_occurrence_id(uint64_t time) const;

  Occurrence get_last_occurrence() const;

  uint64_t get_until() const;

  std::string cron() const;

  std::vector<Occurrence> expand_until(
    uint64_t time);

  void update_occurrence_id(
    uint64_t time,
    const std::string & new_id);

  void update_cron_from(
    uint64_t time,
    uint64_t new_start_time,
    std::string new_cron,
    std::string timezone);

  void update_occurrence_time(
    uint64_t time,
    uint64_t new_start_time);

  void set_id_prefix(
    const std::string & id_prefix);

  void set_until(
    uint64_t time);

  void delete_occurrence(
    uint64_t time);

  // Observer for consolidating with event / dag handling
  template<typename Observer, typename ... ArgsT>
  std::shared_ptr<Observer> make_observer(ArgsT &&... args);

  template<typename Observer>
  void remove_observer(std::shared_ptr<Observer> observer);

private:
  bool _validate_time(uint64_t time, const std::unique_ptr<cron::cronexpr> &) const;

  void _safe_delete(uint64_t time);

  std::unique_ptr<cron::cronexpr> cron_;

  std::string tz_;

  uint64_t until_;

  std::string id_prefix_;

  std::map<uint64_t, std::string> occurrence_ids_;

  std::unordered_set<std::string> exception_ids_;

  std::unordered_set<std::shared_ptr<Series::ObserverBase>> observers_;
};

template<typename Observer, typename ... ArgsT>
std::shared_ptr<Observer> Series::make_observer(ArgsT &&... args)
{
  static_assert(
    std::is_base_of_v<Series::ObserverBase, Observer>,
    "Observer must be derived from Series::ObserverBase"
  );

  // use a local variable to mimic the constructor
  auto ptr = std::make_shared<Observer>(std::forward<ArgsT>(args)...);

  observers_.emplace(std::static_pointer_cast<Series::ObserverBase>(ptr));
  return ptr;
}

template<typename Observer>
void Series::remove_observer(std::shared_ptr<Observer> ptr)
{
  static_assert(
    std::is_base_of_v<Series::ObserverBase, Observer>,
    "Observer must be derived from RuntimeObserverBase"
  );

  observers_.erase(std::static_pointer_cast<Series::ObserverBase>(ptr));
}

}  // namespace data

namespace exception
{

class SeriesEmptyException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  SeriesEmptyException(const char * msg, Args && ... args)
  : ExceptionTemplate(
      ErrorCode::FAILURE | ErrorCode::INVALID_LOGIC | ErrorCode::INVALID_SERIES,
      msg, std::forward<Args>(args) ...)
  {
    add_prefix("SeriesEmptyException: ");
  }
};

class SeriesInvalidCronException : public ExceptionTemplate
{
public:
  template<typename ... Args>
  SeriesInvalidCronException(const char * msg, Args && ... args)
  : ExceptionTemplate(
      ErrorCode::FAILURE | ErrorCode::INVALID_LOGIC | ErrorCode::INVALID_SERIES,
      msg, std::forward<Args>(args) ...)
  {
    add_prefix("SeriesInvalidCronException: ");
  }
};

class SeriesIDException : public IDException
{
public:
  template<typename ... Args>
  SeriesIDException(const char * id, const char * msg, Args && ... args)
  : IDException(
      ErrorCode::FAILURE | ErrorCode::INVALID_ID | ErrorCode::INVALID_SERIES,
      id, msg, std::forward<Args>(args) ...)
  {
    add_prefix("SeriesIDException:\n  ");
  }
};

}  // namespace exception

}  // namespace rmf_scheduler

#endif  // RMF_SCHEDULER__DATA__SERIES_HPP_
