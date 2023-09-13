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

add_library(rmf_scheduler_ros2 SHARED
  src/scheduler_node.cpp
  src/scheduler_executor.cpp
  src/estimate_node.cpp
  src/runtime_node.cpp
  src/scheduler_log_handler.cpp
)

ament_target_dependencies(rmf_scheduler_ros2
  rclcpp
  rmf_scheduler
  rmf_task_msgs
)

target_include_directories(rmf_scheduler_ros2 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

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
    rmf_scheduler_ros2
  EXPORT export_rmf_scheduler_ros2
  LIBRARY DESTINATION lib
)

install(
  TARGETS
    scheduler_node
  EXPORT export_scheduler_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_rmf_scheduler_ros2 HAS_LIBRARY_TARGET)

ament_export_dependencies(
  ${dep_pkgs}
)

ament_package()
