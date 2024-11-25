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
// https://github.com/ros2/rclcpp/blob/master/rclcpp/test/utils/rclcpp_gtest_macros.hpp

#ifndef UTILS__GTEST_MACROS_HPP_
#define UTILS__GTEST_MACROS_HPP_

#include <cstring>
#include <stdexcept>
#include <type_traits>

#include "gtest/gtest.h"

namespace rmf2_scheduler
{
namespace testing
{
namespace details
{

/**
 * \brief Check if two std::exceptions are equal according to their message.
 *
 * If the exception type also derives from rclcpp::Exception, then the next overload is called
 * instead
 */
template<typename T>
::testing::AssertionResult AreThrowableContentsEqual(
  const std::exception & expected, const std::exception & actual,
  const char * expected_exception_expression,
  const char * throwing_expression)
{
  if (std::strcmp(expected.what(), actual.what()) == 0) {
    return ::testing::AssertionSuccess() <<
           "'\nThe contents of the std::exception thrown by the expression\n'" <<
           throwing_expression << "':\n\te.what(): '" << actual.what() <<
           "'\n\nmatch the contents of the expected std::exception\n'" <<
           expected_exception_expression << "'\n\te.what(): '" << expected.what() << "'\n";
  }

  return ::testing::AssertionFailure() <<
         "\nThe contents of the std::exception thrown by the expression\n'" <<
         throwing_expression << "':\n\te.what(): '" << actual.what() <<
         "'\n\ndo not match the contents of the expected std::exception\n'" <<
         expected_exception_expression << "'\n\te.what(): '" << expected.what() << "'\n";
}
}  // namespace details
}  // namespace testing
}  // namespace rmf2_scheduler

/**
 * \def CHECK_THROW_EQ_IMPL
 * \brief Implemented check if statement throws expected exception. don't use directly, use
 * EXPECT_THROW_EQ or ASSERT_THROW_EQ instead.
 */
#define CHECK_THROW_EQ_IMPL(throwing_statement, expected_exception, assertion_result) \
  do { \
    using ExceptionT = decltype(expected_exception); \
    try { \
      throwing_statement; \
      assertion_result = ::testing::AssertionFailure() << \
        "\nExpected the expression:\n\t'" #throwing_statement "'\nto throw: \n\t'" << \
        #expected_exception "'\nbut it did not throw.\n"; \
    } catch (const ExceptionT & e) { \
      assertion_result = \
        rmf2_scheduler::testing::details::AreThrowableContentsEqual<ExceptionT>( \
        expected_exception, e, #expected_exception, #throwing_statement); \
    } catch (const std::exception & e) { \
      assertion_result = ::testing::AssertionFailure() << \
        "\nExpected the expression:\n\t'" #throwing_statement "'\nto throw: \n\t'" << \
        #expected_exception "'\nbut it threw:\n\tType: " << typeid(e).name() << \
        "\n\te.what(): '" << e.what() << "'\n"; \
    } catch (...) { \
      assertion_result = ::testing::AssertionFailure() << \
        "\nExpected the expression:\n\t'" #throwing_statement "'\nto throw: \n\t'" << \
        #expected_exception "'\nbut it threw an unrecognized throwable type.\n"; \
    } \
  } while (0)

/**
 * \def EXPECT_THROW_EQ
 * \brief Check if a statement throws the expected exception type and that the exceptions matches
 *   the expected exception.
 *
 * Like other gtest EXPECT_ macros, this doesn't halt a test and return on failure. Instead it
 * just adds a failure to the current test.
 *
 * See test_gtest_macros.cpp for examples
 */
#define EXPECT_THROW_EQ(throwing_statement, expected_exception) \
  do { \
    ::testing::AssertionResult \
      is_the_result_of_the_throwing_expression_equal_to_the_expected_throwable = \
      ::testing::AssertionSuccess(); \
    CHECK_THROW_EQ_IMPL( \
      throwing_statement, \
      expected_exception, \
      is_the_result_of_the_throwing_expression_equal_to_the_expected_throwable); \
    EXPECT_TRUE(is_the_result_of_the_throwing_expression_equal_to_the_expected_throwable); \
  } while (0)

#endif  // UTILS__GTEST_MACROS_HPP_
