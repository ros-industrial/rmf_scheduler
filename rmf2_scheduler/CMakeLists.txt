cmake_minimum_required(VERSION 3.5)

project(rmf2_scheduler)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(croncpp_vendor REQUIRED)
find_package(taskflow_vendor REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(simpleamqpclient_vendor REQUIRED)

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
  src/cache/action.cpp
  src/cache/event_action.cpp
  src/cache/task_action.cpp
  src/cache/task_handler.cpp
  src/cache/process_action.cpp
  src/cache/process_handler.cpp
  src/cache/schedule_cache.cpp
  src/data/duration.cpp
  src/data/time.cpp
  src/data/node.cpp
  src/data/graph.cpp
  src/data/schedule_action.cpp
  src/data/schedule_change_record.cpp
  src/data/series.cpp
  src/default_log_handler.cpp
  src/executor_data.cpp
  src/http/curl_api.cpp
  src/http/common.cpp
  src/http/connection_curl.cpp
  src/http/connection_fake.cpp
  src/http/memory_stream.cpp
  src/http/request.cpp
  src/http/transport.cpp
  src/http/transport_curl.cpp
  src/http/transport_fake.cpp
  src/log.cpp
  src/scheduler.cpp
  src/scheduler_options.cpp
  src/storage/schedule_stream.cpp
  src/storage/schedule_stream_ld_broker.cpp
  src/system_time_executor.cpp
  src/task_executor_manager.cpp
  src/time_window_lookup.cpp
  src/utils/dag_helper.cpp
  src/utils/data_encoding.cpp
  src/utils/string_utils.cpp
  src/utils/tree_converter.cpp
  src/utils/url_utils.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  taskflow_vendor
  croncpp_vendor
)

target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
  curl
  # nlohmann_json_schema_validator
  # ortools::ortools
  tinyxml2
)

add_executable(dag_to_bt
  examples/dag_to_bt.cpp
)

target_link_libraries(dag_to_bt
  ${PROJECT_NAME}
)

ament_target_dependencies(dag_to_bt
  simpleamqpclient_vendor
)

add_executable(rs_http_request
  examples/rs_http_request.cpp
)

target_link_libraries(rs_http_request
  ${PROJECT_NAME}
)

add_executable(ld_broker_crud
  examples/ld_broker_crud.cpp
)

target_link_libraries(ld_broker_crud
  ${PROJECT_NAME}
)

add_executable(clear_ld_broker
  examples/clear_ld_broker.cpp
)

target_link_libraries(clear_ld_broker
  ${PROJECT_NAME}
)

add_executable(run_scheduler
  examples/run_scheduler.cpp
)

target_link_libraries(run_scheduler
  ${PROJECT_NAME}
)

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

install(TARGETS
  dag_to_bt
  rs_http_request
  ld_broker_crud
  clear_ld_broker
  run_scheduler
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
