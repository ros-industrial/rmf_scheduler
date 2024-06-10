cmake_minimum_required(VERSION 3.5)

project(rmf_scheduler)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_api_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json_schema_validator REQUIRED)
find_package(pluginlib REQUIRED)
find_package(taskflow_vendor REQUIRED)
find_package(croncpp_vendor REQUIRED)
find_package(ortools REQUIRED)
find_package(rmf_notification REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

include(GNUInstallDirs)
include_directories(
  include
  ${CMAKE_BINARY_DIR}/rmf_api_generate_schema_headers/include
)

add_library(${PROJECT_NAME}
  SHARED
  src/cache.cpp
  src/conflict/identifier.cpp
  src/conflict/cp_solver.cpp
  src/data/node.cpp
  src/data/graph.cpp
  src/data/dag.cpp
  src/data/event.cpp
  src/data/events_handler.cpp
  src/data/schedule.cpp
  src/data/series.cpp
  src/error_code.cpp
  src/parser.cpp
  src/runtime/dag_executor.cpp
  src/runtime/system_time_executor.cpp
  src/scheduler.cpp
  src/scheduler_options.cpp
  src/schema_validator.cpp
  src/task/task_plugin.cpp
  src/task/builder.cpp
  src/task/estimate_interface.cpp
  src/task/estimator.cpp
  src/task/execution_interface.cpp
  src/task/executor.cpp
  src/utils/system_time_utils.cpp
  src/log.cpp
  src/default_log_handler.cpp
  src/window.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  taskflow_vendor
  croncpp_vendor
  pluginlib
  rclcpp
  rmf_notification
)

target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
  nlohmann_json_schema_validator
  ortools::ortools
)

rmf_api_generate_schema_headers(
  PACKAGE rmf_scheduler
  SCHEMAS_DIR schemas
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
  rmf_notification
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
