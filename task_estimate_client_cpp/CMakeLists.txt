cmake_minimum_required(VERSION 3.5.0)
project(task_estimate_client_cpp)

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
  rclcpp_components
  rmf_task_msgs
  rmf_api_msgs
  nlohmann_json
  nlohmann_json_schema_validator
)

foreach(pkg ${dep_pkgs})
  find_package(${pkg} REQUIRED)
endforeach()

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/task_estimate_client.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rmf_task_msgs
)

target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
  rmf_api_msgs::rmf_api_msgs
  nlohmann_json_schema_validator
)

# Test script for estimating GoToPlace task
add_executable(test_go_to_place
  test/test_go_to_place.cpp
)

target_link_libraries(test_go_to_place
  task_estimate_client_cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    test_go_to_place
    task_estimate_client_cpp
  EXPORT task_estimate_client_cpp
  RUNTIME DESTINATION lib/task_estimate_client_cpp
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_dependencies(
  ${dep_pkgs}
)

ament_export_libraries(
  task_estimate_client_cpp
)

ament_package()
