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
// https://github.com/ros2/rclcpp/blob/master/rclcpp/src/rclcpp/duration.cpp

#include <cmath>
#include <stdexcept>

#include "rmf2_scheduler/data/duration.hpp"
#include "rmf2_scheduler/utils/utils.hpp"

namespace rmf2_scheduler
{

namespace data
{


Duration::Duration(int32_t seconds, uint32_t nanoseconds)
{
  duration_value_ = static_cast<int64_t>(seconds) * 1000LL * 1000LL * 1000LL;
  duration_value_ += nanoseconds;
}

Duration::Duration(std::chrono::nanoseconds nanoseconds)
{
  duration_value_ = nanoseconds.count();
}

Duration::Duration(const Duration & rhs) = default;


Duration::Duration(const rs_duration_value_t & duration)
: duration_value_(duration)
{
  // noop
}

Duration &
Duration::operator=(const Duration & rhs) = default;

bool
Duration::operator==(const Duration & rhs) const
{
  return duration_value_ == rhs.duration_value_;
}

bool
Duration::operator!=(const Duration & rhs) const
{
  return duration_value_ != rhs.duration_value_;
}

bool
Duration::operator<(const Duration & rhs) const
{
  return duration_value_ < rhs.duration_value_;
}

bool
Duration::operator<=(const Duration & rhs) const
{
  return duration_value_ <= rhs.duration_value_;
}

bool
Duration::operator>=(const Duration & rhs) const
{
  return duration_value_ >= rhs.duration_value_;
}

bool
Duration::operator>(const Duration & rhs) const
{
  return duration_value_ > rhs.duration_value_;
}

Duration
Duration::operator+(const Duration & rhs) const
{
  if (utils::add_will_overflow(rhs.duration_value_, duration_value_)) {
    throw std::overflow_error("addition leads to int64_t overflow");
  }

  if (utils::add_will_underflow(rhs.duration_value_, duration_value_)) {
    throw std::underflow_error("addition leads to int64_t underflow");
  }

  return Duration::from_nanoseconds(
    duration_value_ + rhs.duration_value_);
}

Duration &
Duration::operator+=(const Duration & rhs)
{
  *this = *this + rhs;
  return *this;
}

Duration
Duration::operator-(const Duration & rhs) const
{
  if (utils::sub_will_overflow(duration_value_, rhs.duration_value_)) {
    throw std::overflow_error("duration subtraction leads to int64_t overflow");
  }

  if (utils::sub_will_underflow(duration_value_, rhs.duration_value_)) {
    throw std::underflow_error("duration subtraction leads to int64_t underflow");
  }

  return Duration::from_nanoseconds(
    duration_value_ - rhs.duration_value_);
}

Duration &
Duration::operator-=(const Duration & rhs)
{
  *this = *this - rhs;
  return *this;
}

void
bounds_check_duration_scale(int64_t dns, double scale, uint64_t max)
{
  auto abs_dns = static_cast<uint64_t>(std::abs(dns));
  auto abs_scale = std::abs(scale);
  if (abs_scale > 1.0 && abs_dns >
    static_cast<uint64_t>(static_cast<long double>(max) / static_cast<long double>(abs_scale)))
  {
    if ((dns > 0 && scale > 0) || (dns < 0 && scale < 0)) {
      throw std::overflow_error("duration scaling leads to int64_t overflow");
    } else {
      throw std::underflow_error("duration scaling leads to int64_t underflow");
    }
  }
}

Duration
Duration::operator*(double scale) const
{
  if (!std::isfinite(scale)) {
    throw std::runtime_error("abnormal scale in Duration");
  }
  bounds_check_duration_scale(
    this->duration_value_,
    scale,
    std::numeric_limits<rs_duration_value_t>::max());
  long double scale_ld = static_cast<long double>(scale);
  return Duration::from_nanoseconds(
    static_cast<rs_duration_value_t>(
      static_cast<long double>(duration_value_) * scale_ld));
}

Duration &
Duration::operator*=(double scale)
{
  *this = *this * scale;
  return *this;
}

rs_duration_value_t
Duration::nanoseconds() const
{
  return duration_value_;
}

Duration
Duration::max()
{
  return Duration(std::numeric_limits<int32_t>::max(), 999999999);
}

double
Duration::seconds() const
{
  return std::chrono::duration<double>(std::chrono::nanoseconds(duration_value_)).count();
}

Duration
Duration::from_seconds(double seconds)
{
  Duration ret;
  ret.duration_value_ = static_cast<int64_t>(seconds * 1000LL * 1000LL * 1000LL);
  return ret;
}

Duration
Duration::from_nanoseconds(rs_duration_value_t nanoseconds)
{
  Duration ret;
  ret.duration_value_ = nanoseconds;
  return ret;
}

}  // namespace data

}  // namespace rmf2_scheduler
