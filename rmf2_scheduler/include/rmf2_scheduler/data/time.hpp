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
// https://github.com/ros2/rclcpp/blob/master/rclcpp/include/rclcpp/time.hpp

#ifndef RMF2_SCHEDULER__DATA__TIME_HPP_
#define RMF2_SCHEDULER__DATA__TIME_HPP_

#include <cstdint>
#include <string>

#include "rmf2_scheduler/data/duration.hpp"

/// A single point in time, measured in nanoseconds since the Unix epoch.
typedef int64_t rs_time_point_value_t;

namespace rmf2_scheduler
{

namespace data
{

class Time
{
public:
  Time();
  /// Time constructor
  /**
   * Initializes the time values for seconds and nanoseconds individually.
   * Large values for nanoseconds are wrapped automatically with the remainder added to seconds.
   * Both inputs must be integers.
   *
   * \param seconds part of the time in seconds since time epoch
   * \param nanoseconds part of the time in nanoseconds since time epoch
   */
  Time(int32_t seconds, uint32_t nanoseconds);

  /// Time constructor
  /**
   * \param nanoseconds since time epoch
   * \throws std::runtime_error if nanoseconds are negative
   */
  explicit Time(int64_t nanoseconds);

  /// Time constructor
  /**
   * This constructor matches any std::chrono time_point value.
   * However, it is not wise to use different clock.
   */
  template<class Clock, class DurationType = typename Clock::duration>
  // cppcheck-suppress noExplicitConstructor
  Time(
    const std::chrono::time_point<Clock, DurationType> & time_point
  )  // NOLINT(runtime/explicit)
  : time_value_(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        time_point.time_since_epoch()
      ).count()
  )
  {}

  /// Copy constructor
  Time(const Time & rhs);

  /// Move constructor
  Time(Time && rhs) noexcept;

  /// Time destructor
  virtual ~Time();

  /// Return a rs_time_point_value object based
  operator rs_time_point_value_t() const;

  /**
   * Copy assignment operator
   */
  Time &
  operator=(const Time & rhs);

  /**
   * Move assignment operator
   */
  Time &
  operator=(Time && rhs) noexcept;

  bool
  operator==(const Time & rhs) const;

  bool
  operator!=(const Time & rhs) const;

  bool
  operator<(const Time & rhs) const;

  bool
  operator<=(const Time & rhs) const;

  bool
  operator>=(const Time & rhs) const;

  bool
  operator>(const Time & rhs) const;

  /**
   * \throws std::overflow_error if addition leads to overflow
   */
  Time
  operator+(const Duration & rhs) const;

  /**
   * \throws std::overflow_error if addition leads to overflow
   */
  Duration
  operator-(const Time & rhs) const;

  /**
   * \throws std::overflow_error if addition leads to overflow
   */
  Time
  operator-(const Duration & rhs) const;

  /**
   * \throws std::overflow_error if addition leads to overflow
   */
  Time &
  operator+=(const Duration & rhs);

  /**
   * \throws std::overflow_error if addition leads to overflow
   */
  Time &
  operator-=(const Duration & rhs);

  /// Get the nanoseconds since epoch
  /**
   * \return the nanoseconds since epoch as a uint32_t structure.
   */
  int64_t
  nanoseconds() const;

  /// Get the maximum representable value.
  /**
   * \return the maximum representable value
   */
  static Time max();

  /// Get the seconds since epoch
  /**
   * \warning Depending on sizeof(double) there could be significant precision loss.
   * When an exact time is required use nanoseconds() instead.
   *
   * \return the seconds since epoch as a floating point number.
   */
  double
  seconds() const;

  /// Convert Time into a std::chrono::time_point
  template<class Clock, class DurationType = typename Clock::duration>
  std::chrono::time_point<Clock, DurationType>
  to_chrono() const
  {
    return std::chrono::time_point<Clock, DurationType>(
      std::chrono::nanoseconds(time_value_)
    );
  }

  /// Convert Time into a localtime string
  /**
   * \param fmt localtime format.
   */
  char * to_localtime(const std::string & fmt = "%b %d %H:%M:%S %Y") const;

  /// Convert Time into a localtime string in ISO format
  char * to_ISOtime() const;

  /// Create a Time object from a string representing localtime
  static Time from_localtime(
    const std::string & localtime,
    const std::string & fmt = "%b %d %H:%M:%S %Y"
  );

  /// Create a Time object from a string representing localtime in ISO format
  static Time from_ISOtime(
    const std::string & localtime
  );

private:
  rs_time_point_value_t time_value_;
};

/**
 * \throws std::overflow_error if addition leads to overflow
 */
Time
operator+(const Duration & lhs, const Time & rhs);

}  // namespace data

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__DATA__TIME_HPP_
