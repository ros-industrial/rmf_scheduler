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

#ifndef RMF2_SCHEDULER__DATA__SCHEDULE_CHANGE_RECORD_HPP_
#define RMF2_SCHEDULER__DATA__SCHEDULE_CHANGE_RECORD_HPP_

#include <string>
#include <vector>

namespace rmf2_scheduler
{

namespace data
{

namespace record_data_type
{
extern const char EVENT[];
extern const char TASK[];
extern const char PROCESS[];
extern const char SERIES[];
}

namespace record_action_type
{
extern const char ADD[];
extern const char UPDATE[];
extern const char DELETE[];
}

struct ChangeAction
{
  std::string id;
  std::string action;

  inline bool operator==(const ChangeAction & rhs) const
  {
    if (this->id != rhs.id) {return false;}
    if (this->action != rhs.action) {return false;}
    return true;
  }

  inline bool operator!=(const ChangeAction & rhs) const
  {
    return !(*this == rhs);
  }
};

class ScheduleChangeRecord
{
public:
  ScheduleChangeRecord() = default;
  virtual ~ScheduleChangeRecord() {}

  /// Add record
  /**
   * \param[in] type record type
   * \param[in] ids IDs changed
   */
  void add(
    const std::string & type,
    const std::vector<ChangeAction> & change_actions
  );

  /// Retrieve record
  /**
   * \param[in] type record type
   * \returns record
   */
  const std::vector<ChangeAction> & get(
    const std::string & type
  ) const;

  /// Squash multiple records into 1 record
  /**
   * \param[in] records a list of record in order
   * \returns the squashed record
   */
  static ScheduleChangeRecord squash(const std::vector<ScheduleChangeRecord> & records);

private:
  std::vector<ChangeAction> event_changes_;
  std::vector<ChangeAction> process_changes_;
  std::vector<ChangeAction> series_changes_;
};

}  // namespace data
}  // namespace rmf2_scheduler
#endif  // RMF2_SCHEDULER__DATA__SCHEDULE_CHANGE_RECORD_HPP_
