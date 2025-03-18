// Copyright 2024 ROS Industrial Consortium Asia Pacific
// Copyright 2017 Open Source Robotics Foundation, Inc.
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
//
// Original file taken from:
// https://github.com/ros2/rclcpp/blob/master/rclcpp/src/rclcpp/time.cpp

#include <cstdint>
#include <cstring>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <stdexcept>

#include "rmf2_scheduler/data/time.hpp"
#include "rmf2_scheduler/utils/utils.hpp"


namespace rmf2_scheduler
{

namespace data
{

Time::Time()
: time_value_(0) {}

Time::Time(int32_t seconds, uint32_t nanoseconds)
{
  time_value_ = static_cast<int64_t>(seconds) * 1000LL * 1000LL * 1000LL;
  time_value_ += nanoseconds;
}


Time::Time(int64_t nanoseconds)
{
  if (nanoseconds < 0) {
    throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
  }

  time_value_ = nanoseconds;
}

Time::Time(const Time & rhs) = default;

Time::Time(Time && rhs) noexcept = default;

Time::~Time() = default;

Time::operator rs_time_point_value_t() const {
  return time_value_;
}

Time &
Time::operator=(const Time & rhs) = default;

Time &
Time::operator=(Time && rhs) noexcept = default;

bool
Time::operator==(const Time & rhs) const
{
  return time_value_ == rhs.time_value_;
}

bool
Time::operator!=(const Time & rhs) const
{
  return !(*this == rhs);
}

bool
Time::operator<(const Time & rhs) const
{
  return time_value_ < rhs.time_value_;
}

bool
Time::operator<=(const Time & rhs) const
{
  return time_value_ <= rhs.time_value_;
}

bool
Time::operator>=(const Time & rhs) const
{
  return time_value_ >= rhs.time_value_;
}

bool
Time::operator>(const Time & rhs) const
{
  return time_value_ > rhs.time_value_;
}

Time
Time::operator+(const Duration & rhs) const
{
  if (utils::add_will_overflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::overflow_error("addition leads to int64_t overflow");
  }
  if (utils::add_will_underflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::underflow_error("addition leads to int64_t underflow");
  }
  return Time(this->nanoseconds() + rhs.nanoseconds());
}

Duration
Time::operator-(const Time & rhs) const
{
  if (utils::sub_will_overflow(time_value_, rhs.time_value_)) {
    throw std::overflow_error("time subtraction leads to int64_t overflow");
  }

  if (utils::sub_will_underflow(time_value_, rhs.time_value_)) {
    throw std::underflow_error("time subtraction leads to int64_t underflow");
  }

  return Duration::from_nanoseconds(time_value_ - rhs.time_value_);
}

Time
Time::operator-(const Duration & rhs) const
{
  if (utils::sub_will_overflow(time_value_, rhs.nanoseconds())) {
    throw std::overflow_error("time subtraction leads to int64_t overflow");
  }
  if (utils::sub_will_underflow(time_value_, rhs.nanoseconds())) {
    throw std::underflow_error("time subtraction leads to int64_t underflow");
  }

  return Time(time_value_ - rhs.nanoseconds());
}

int64_t
Time::nanoseconds() const
{
  return time_value_;
}

double
Time::seconds() const
{
  return std::chrono::duration<double>(std::chrono::nanoseconds(time_value_)).count();
}

Time
operator+(const Duration & lhs, const Time & rhs)
{
  if (utils::add_will_overflow(rhs.nanoseconds(), lhs.nanoseconds())) {
    throw std::overflow_error("addition leads to int64_t overflow");
  }
  if (utils::add_will_underflow(rhs.nanoseconds(), lhs.nanoseconds())) {
    throw std::underflow_error("addition leads to int64_t underflow");
  }
  return Time(lhs.nanoseconds() + rhs.nanoseconds());
}

Time &
Time::operator+=(const Duration & rhs)
{
  if (utils::add_will_overflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::overflow_error("addition leads to int64_t overflow");
  }
  if (utils::add_will_underflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::underflow_error("addition leads to int64_t underflow");
  }

  time_value_ += rhs.nanoseconds();
  if (time_value_ < 0) {
    throw std::runtime_error("cannot store a negative time point in Time");
  }

  return *this;
}

Time &
Time::operator-=(const Duration & rhs)
{
  if (utils::sub_will_overflow(time_value_, rhs.nanoseconds())) {
    throw std::overflow_error("time subtraction leads to int64_t overflow");
  }
  if (utils::sub_will_underflow(time_value_, rhs.nanoseconds())) {
    throw std::underflow_error("time subtraction leads to int64_t underflow");
  }

  time_value_ -= rhs.nanoseconds();
  if (time_value_ < 0) {
    throw std::runtime_error("cannot store a negative time point in Time");
  }

  return *this;
}

Time
Time::max()
{
  return Time(std::numeric_limits<int32_t>::max(), 999999999);
}

char * Time::to_localtime(
  const std::string & fmt
) const
{
  time_t t = seconds();
  std::tm ltime;
  localtime_r(&t, &ltime);
  static char mbstr[50];
  std::strftime(mbstr, sizeof(mbstr), fmt.c_str(), &ltime);
  return mbstr;
}

char * Time::to_ISOtime() const
{
  // to format [yyyy-mm-ddThh:mm:ss[.mmm]]
  char * iso_time_base = to_localtime("%FT%T");

  int truncated_seconds = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::nanoseconds(time_value_)
  ).count();
  double delta_ns = seconds() - truncated_seconds;

  static char iso_time[40];
  std::memcpy(iso_time, iso_time_base, sizeof(iso_time));

  if (delta_ns > 0) {
    std::stringstream ss;
    ss << std::setprecision(3) << std::fixed << delta_ns;
    snprintf(iso_time, sizeof(iso_time), "%s%sZ", iso_time_base, ss.str().substr(1).c_str());
  } else {
    snprintf(iso_time, sizeof(iso_time), "%sZ", iso_time_base);
  }

  return iso_time;
}

data::Time Time::from_localtime(
  const std::string & localtime,
  const std::string & fmt)
{
  std::tm t = {};
  std::istringstream ss(localtime);
  ss >> std::get_time(&t, fmt.c_str());
  if (ss.fail()) {
    throw std::invalid_argument(
            "from_localtime failed, [" + localtime + "] "
            "doesn't follow [" + fmt + "]."
    );
  }
  time_t t_s = std::mktime(&t);
  return data::Time(t_s, 0);
}

data::Time Time::from_ISOtime(
  const std::string & localtime
)
{
  data::Time base_time = from_localtime(localtime, "%Y-%m-%dT%H:%M:%S");
  if (localtime.size() <= 20) {
    return base_time;
  }

  double delta_ns = stod(localtime.substr(19, 4));
  return base_time + Duration::from_seconds(delta_ns);
}


}  // namespace data

}  // namespace rmf2_scheduler
