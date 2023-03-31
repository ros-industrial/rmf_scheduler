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

include(GNUInstallDirs)

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

add_library(task_estimate_client_cpp SHARED
  src/task_estimate_client.cpp)

target_link_libraries(task_estimate_client_cpp
  PUBLIC
    ${rclcpp_LIBRARIES}
    ${rmf_task_msgs_LIBRARIES}
    nlohmann_json::nlohmann_json
    rmf_api_msgs::rmf_api_msgs
    nlohmann_json_schema_validator
)

target_include_directories(task_estimate_client_cpp
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rclcpp_INCLUDE_DIRS}
    ${rmf_task_msgs_INCLUDE_DIRS}
  PRIVATE
    ${rmf_api_msgs_INCLUDE_DIRS}
)

add_executable(test_go_to_place
  test/test_go_to_place.cpp
)

target_link_libraries(test_go_to_place
  PRIVATE
    task_estimate_client_cpp
)

target_include_directories(test_go_to_place
  PRIVATE
    task_estimate_client_cpp
)

message(WARNING ${nlohmann_json_schema_validator_INCLUDE_DIRS})

# add_executable(task_estimate_requestor
#   src/main.cpp)

# target_link_libraries(task_estimate_requestor
#   PRIVATE
#     rmf_fleet_adapter)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(
  include
)

ament_export_dependencies(
  ${dep_pkgs}
)

ament_export_libraries()

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
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_package()
