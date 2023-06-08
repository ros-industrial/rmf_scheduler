cmake_minimum_required(VERSION 3.8)
project(rmf_scheduler_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(dep_pkgs
  rclcpp
  rmf_task_msgs
  rmf_scheduler
  nlohmann_json
  nlohmann_json_schema_validator
)

foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

include_directories(
  include
)

add_library(rmf_scheduler_ros2 SHARED
  src/rmf_scheduler_node.cpp)

ament_target_dependencies(rmf_scheduler_ros2
  rclcpp
  rmf_scheduler
  rmf_task_msgs)

target_link_libraries(rmf_scheduler_ros2
  nlohmann_json::nlohmann_json
)

add_executable(scheduler_node src/main.cpp)

target_link_libraries(scheduler_node
  rmf_scheduler_ros2
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    scheduler_node
    rmf_scheduler_ros2
  EXPORT rmf_scheduler_ros2
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

ament_package()
