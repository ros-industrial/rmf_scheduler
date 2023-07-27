cmake_minimum_required(VERSION 3.5.0)
project(rmf_scheduler_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(dep_pkgs
  rclcpp
  rmf_task_msgs
  rmf_api_msgs
  rmf_scheduler
  pluginlib
  nlohmann_json
  nlohmann_json_schema_validator
)

foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()


include_directories(
  include
)

add_library(rmf_robot_task_estimate_client SHARED
  src/robot_task_estimate_client.cpp
)

ament_target_dependencies(rmf_robot_task_estimate_client
  rclcpp
  rmf_scheduler
  rmf_task_msgs
  pluginlib
)

target_link_libraries(rmf_robot_task_estimate_client
  nlohmann_json::nlohmann_json
  rmf_api_msgs::rmf_api_msgs
  nlohmann_json_schema_validator
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(TARGETS
  rmf_robot_task_estimate_client
  EXPORT export_rmf_scheduler_plugins
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_rmf_scheduler_plugins HAS_LIBRARY_TARGET)
ament_export_include_directories(
  include
)

ament_export_dependencies(
  ${dep_pkgs}
)

ament_package()
