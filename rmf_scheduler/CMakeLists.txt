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
  src/conflict_identifier.cpp
  src/conflict_resolver/cp_solver.cpp
  src/dag.cpp
  src/dag_executor.cpp
  src/error_code.cpp
  src/events_handler.cpp
  src/parser.cpp
  src/runtime_interface.cpp
  src/runtime.cpp
  src/schedule.cpp
  src/scheduler.cpp
  src/schema_validator.cpp
  src/series.cpp
  src/system_time_executor.cpp
  src/system_time_utils.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  taskflow_vendor
  croncpp_vendor
  pluginlib
  rclcpp
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
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
