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

#ifndef RMF2_SCHEDULER__DATA__SERIES_HPP_
#define RMF2_SCHEDULER__DATA__SERIES_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rmf2_scheduler/data/time.hpp"
#include "rmf2_scheduler/data/occurrence.hpp"
#include "rmf2_scheduler/macros.hpp"

namespace cron
{

class cronexpr;

}  // namespace cron

namespace rmf2_scheduler
{

namespace data
{

class Series
{
public:
  RS_SMART_PTR_DEFINITIONS(Series)


  class ObserverBase
  {
public:
    virtual ~ObserverBase() {}
    virtual void on_add_occurrence(
      const Occurrence & ref_occurrence,
      const Occurrence & new_occurrence
    ) = 0;

    virtual void on_update_occurrence(
      const Occurrence & old_occurrence,
      const Time & new_time) = 0;

    virtual void on_delete_occurrence(
      const Occurrence & occurrence) = 0;
  };

  Series();
  Series(const Series &);
  Series & operator=(const Series &);
  Series(Series &&);
  Series & operator=(Series && rhs);

  virtual ~Series();

  /// Constructor
  /**
   * Create a series using a single occurrence
   *
   * \param id series ID
   * \param type series type
   * \param occurrence occurrence to use as the first event
   * \param cron cron string
   * \param timezone series timezone
   * \param until series until
   * \throws std::invalid_argument if cron is invalid.
   */
  explicit Series(
    const std::string & id,
    const std::string & type,
    const Occurrence & occurrence,
    const std::string & cron,
    const std::string & timezone,
    const Time & until = Time::max()
  );

  /// Constructor
  /**
   * Create a series from multiple occurrences
   *
   * \param id series ID
   * \param occurrences occurrences to load into the series
   * \param cron cron string
   * \param timezone series timezone
   * \param until series until
   * \param exception_ids occurrences that are mark as exceptions
   * \throws std::invalid_argument if cron is invalid.
   */
  explicit Series(
    const std::string & id,
    const std::string & type,
    const std::vector<Occurrence> & occurrences,
    const std::string & cron,
    const std::string & timezone,
    const Time & until = Time::max(),
    const std::vector<std::string> & exception_ids = {}
  );

  explicit operator bool() const;

  const std::string & id() const;

  const std::string & type() const;

  std::vector<Occurrence> occurrences() const;

  Occurrence get_occurrence(const Time & time) const;

  Occurrence get_occurrence(const std::string & id) const;

  Occurrence get_last_occurrence() const;

  Occurrence get_first_occurrence() const;

  const Time & until() const;

  std::string cron() const;

  std::string tz() const;

  const std::unordered_set<std::string> & exception_ids() const;

  std::vector<std::string> exception_ids_ordered() const;

  std::vector<Occurrence> expand_until(
    const Time & time
  );

  void update_id(const std::string & new_id);

  void update_occurrence(
    const Time & time,
    const std::string & new_id
  );

  void update_occurrence(
    const Time & time,
    const Time & new_start_time
  );

  void update_cron_from(
    const Time & time,
    const Time & new_start_time,
    std::string new_cron,
    std::string timezone
  );

  void update_until(const Time & time);

  void delete_occurrence(const Time & time);

  // Observer for consolidating with event / dag handling
  template<typename Observer, typename ... ArgsT>
  std::shared_ptr<Observer> make_observer(ArgsT &&... args);

  template<typename Observer>
  void remove_observer(std::shared_ptr<Observer> observer);

  inline bool operator==(const Series & rhs) const
  {
    if (this->id_ != rhs.id_) {return false;}
    if (this->type_ != rhs.type_) {return false;}
    if (this->cron() != rhs.cron()) {return false;}
    if (this->tz_ != rhs.tz_) {return false;}
    if (this->until_ != rhs.until_) {return false;}
    if (this->occurrence_lookup_ != rhs.occurrence_lookup_) {return false;}
    if (this->exception_ids_ != rhs.exception_ids_) {return false;}
    if (this->observers_ != rhs.observers_) {return false;}
    return true;
  }

  inline bool operator!=(const Series & rhs) const
  {
    return !(*this == rhs);
  }

private:
  bool _validate_time(const Time & time, const std::unique_ptr<cron::cronexpr> &) const;

  void _safe_delete(rs_time_point_value_t time);

  std::string id_;

  std::string type_;

  std::unique_ptr<cron::cronexpr> cron_;

  std::string tz_;

  Time until_;

  std::map<rs_time_point_value_t, std::string> occurrence_lookup_;

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

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__DATA__SERIES_HPP_
