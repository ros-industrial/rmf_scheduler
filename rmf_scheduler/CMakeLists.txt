cmake_minimum_required(VERSION 3.5)

project(rmf_scheduler)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/events_handler.cpp
  src/system_time_executor.cpp
  src/system_time_utils.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_dependencies(
  rclcpp
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
