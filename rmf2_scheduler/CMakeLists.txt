cmake_minimum_required(VERSION 3.5)

project(rmf2_scheduler)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(croncpp_vendor REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

include(GNUInstallDirs)
include_directories(
  include
)

add_library(${PROJECT_NAME}
  SHARED
  src/data/duration.cpp
  src/data/time.cpp
  src/data/node.cpp
  src/data/graph.cpp
  src/data/series.cpp
)

ament_target_dependencies(${PROJECT_NAME}
#  taskflow_vendor
  croncpp_vendor
)

target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
  # nlohmann_json_schema_validator
  # ortools::ortools
  # tinyxml2
)
#
# rmf_api_generate_schema_headers(
#   PACKAGE rmf_scheduler
#   SCHEMAS_DIR schemas
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

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

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
