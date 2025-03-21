// Copyright 2024 ROS Industrial Consortium Asia Pacific
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

#ifndef RMF2_SCHEDULER__UTILS__UTILS_HPP_
#define RMF2_SCHEDULER__UTILS__UTILS_HPP_

#include <iterator>
#include <iomanip>
#include <limits>
#include <memory>

namespace rmf2_scheduler
{

namespace utils
{
/// Safely check if addition will overflow.
/**
 * The type of the operands, T, should have defined
 * std::numeric_limits<T>::max(), `>`, `<` and `-` operators.
 *
 * \param[in] x is the first addend.
 * \param[in] y is the second addend.
 * \tparam T is type of the operands.
 * \return True if the x + y sum is greater than T::max value.
 */
template<typename T>
bool
add_will_overflow(const T x, const T y)
{
  return (y > 0) && (x > (std::numeric_limits<T>::max() - y));
}

/// Safely check if addition will underflow.
/**
 * The type of the operands, T, should have defined
 * std::numeric_limits<T>::min(), `>`, `<` and `-` operators.
 *
 * \param[in] x is the first addend.
 * \param[in] y is the second addend.
 * \tparam T is type of the operands.
 * \return True if the x + y sum is less than T::min value.
 */
template<typename T>
bool
add_will_underflow(const T x, const T y)
{
  return (y < 0) && (x < (std::numeric_limits<T>::min() - y));
}

/// Safely check if subtraction will overflow.
/**
 * The type of the operands, T, should have defined
 * std::numeric_limits<T>::max(), `>`, `<` and `+` operators.
 *
 * \param[in] x is the minuend.
 * \param[in] y is the subtrahend.
 * \tparam T is type of the operands.
 * \return True if the difference `x - y` sum is grater than T::max value.
 */
template<typename T>
bool
sub_will_overflow(const T x, const T y)
{
  return (y < 0) && (x > (std::numeric_limits<T>::max() + y));
}

/// Safely check if subtraction will underflow.
/**
 * The type of the operands, T, should have defined
 * std::numeric_limits<T>::min(), `>`, `<` and `+` operators.
 *
 * \param[in] x is the minuend.
 * \param[in] y is the subtrahend.
 * \tparam T is type of the operands.
 * \return True if the difference `x - y` sum is less than T::min value.
 */
template<typename T>
bool
sub_will_underflow(const T x, const T y)
{
  return (y > 0) && (x < (std::numeric_limits<T>::min() + y));
}

template<class Container, class InputConstItr>
auto remove_iterator_const(
  Container & c,
  const InputConstItr & itr_const
) -> decltype(std::begin(c))
{
  static_assert(std::is_same_v<InputConstItr, decltype(std::cbegin(c))>);

  auto itr = std::begin(c);
  std::advance(
    itr,
    std::distance<InputConstItr>(
      std::cbegin(c),
      itr_const
    )
  );

  return itr;
}

template<class T>
bool is_deep_equal(
  const std::shared_ptr<T> & lhs,
  const std::shared_ptr<T> & rhs
)
{
  if (lhs == rhs) {
    return true;
  }

  if (lhs && rhs && *lhs == *rhs) {
    return true;
  }

  return false;
}

}  // namespace utils

}  // namespace rmf2_scheduler

#endif  // RMF2_SCHEDULER__UTILS__UTILS_HPP_
