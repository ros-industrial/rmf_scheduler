cmake_minimum_required(VERSION 3.5)

project(rmf_notification)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(websocketpp REQUIRED)
find_package(cpprestsdk REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rmf_scheduler REQUIRED)
find_package(OpenSSL REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

include_directories(
  include
)

add_library(${PROJECT_NAME}
  SHARED
  src/notification_manager.cpp
  src/message.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  websocketpp
  rclcpp
  rmf_scheduler
)

target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
  cpprestsdk::cpprest
  OpenSSL::SSL
)

add_executable(${PROJECT_NAME}_example_node
  example/notification_node.cpp
)

target_link_libraries(${PROJECT_NAME}_example_node
  ${PROJECT_NAME}
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

install(
  TARGETS
  ${PROJECT_NAME}_example_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
