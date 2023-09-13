cmake_minimum_required(VERSION 3.5.0)
project(rmf_scheduler_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

add_library(rmf_scheduler_plugins SHARED
  src/robot_task_builder.cpp
  src/robot_task_estimate_client.cpp
  src/robot_task_execution_client.cpp
)

ament_target_dependencies(rmf_scheduler_plugins
  rclcpp
  rmf_scheduler
  rmf_task_msgs
  pluginlib
)

target_link_libraries(rmf_scheduler_plugins
  nlohmann_json::nlohmann_json
  rmf_api_msgs::rmf_api_msgs
  nlohmann_json_schema_validator
)

target_include_directories(rmf_scheduler_plugins PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(rmf_scheduler plugins_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

install(TARGETS
  rmf_scheduler_plugins
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

ament_export_dependencies(
  ${dep_pkgs}
)

ament_package()
