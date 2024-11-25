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
// https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/duration.hpp

#ifndef RMF2_SCHEDULER__DATA__DURATION_HPP_
#define RMF2_SCHEDULER__DATA__DURATION_HPP_

#include <cstdint>
#include <chrono>

/// A duration of time, measured in nanoseconds.
typedef int64_t rs_duration_value_t;

namespace rmf2_scheduler
{

namespace data
{

class Duration
{
public:
  Duration() = default;

  /// Duration constructor.
  /**
   * Initializes the time values for seconds and nanoseconds individually.
   * Large values for nsecs are wrapped automatically with the remainder added to secs.
   * Both inputs must be integers.
   * Seconds can be negative.
   *
   * \param seconds time in seconds
   * \param nanoseconds time in nanoseconds
   */
  Duration(int32_t seconds, uint32_t nanoseconds);

  /// Construct duration from the specified std::chrono::nanoseconds.
  explicit Duration(std::chrono::nanoseconds nanoseconds);

  // This constructor matches any std::chrono value other than nanoseconds
  // intentionally not using explicit to create a conversion constructor
  template<class Rep, class Period>
  // cppcheck-suppress noExplicitConstructor
  Duration(const std::chrono::duration<Rep, Period> & duration)  // NOLINT(runtime/explicit)
  : Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(duration))
  {}

  /// Time constructor
  /**
   * \param duration rcl_duration_t structure to copy.
   */
  explicit Duration(const rs_duration_value_t & duration);

  Duration(const Duration & rhs);

  virtual ~Duration() = default;

  // cppcheck-suppress operatorEq // this is a false positive from cppcheck
  Duration &
  operator=(const Duration & rhs);

  bool
  operator==(const Duration & rhs) const;

  bool
  operator!=(const Duration & rhs) const;

  bool
  operator<(const Duration & rhs) const;

  bool
  operator<=(const Duration & rhs) const;

  bool
  operator>=(const Duration & rhs) const;

  bool
  operator>(const Duration & rhs) const;

  Duration
  operator+(const Duration & rhs) const;

  Duration & operator+=(const Duration & rhs);

  Duration
  operator-(const Duration & rhs) const;

  Duration & operator-=(const Duration & rhs);

  /// Get the maximum representable value.
  /**
   * \return the maximum representable value
   */
  static
  Duration
  max();

  Duration
  operator*(double scale) const;

  Duration &
  operator*=(double scale);

  /// Get duration in nanosecods
  /**
   * \return the duration in nanoseconds as a rcl_duration_value_t.
   */
  rs_duration_value_t
  nanoseconds() const;

  /// Get duration in seconds
  /**
   * \warning Depending on sizeof(double) there could be significant precision loss.
   *   When an exact time is required use nanoseconds() instead.
   * \return the duration in seconds as a floating point number.
   */
  double
  seconds() const;

  /// Create a duration object from a floating point number representing seconds
  static Duration
  from_seconds(double seconds);

  /// Create a duration object from an integer number representing nanoseconds
  static Duration
  from_nanoseconds(rs_duration_value_t nanoseconds);

  /// Convert Duration into a std::chrono::Duration.
  template<class DurationT>
  DurationT
  to_chrono() const
  {
    return std::chrono::duration_cast<DurationT>(std::chrono::nanoseconds(this->nanoseconds()));
  }

private:
  rs_duration_value_t duration_value_;
};

}  // namespace data

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__DATA__DURATION_HPP_
