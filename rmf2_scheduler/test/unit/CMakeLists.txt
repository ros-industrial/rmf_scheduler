find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

add_definitions(-DTEST_RESOURCES_DIRECTORY="${TEST_RESOURCES_DIRECTORY}")

# DATA
ament_add_gtest(test_data_json_serializer
  data/test_json_serializer.cpp
)
target_link_libraries(test_data_json_serializer ${PROJECT_NAME})

ament_add_gtest(test_data_json_macro
  data/test_json_macro.cpp
)
target_link_libraries(test_data_json_macro ${PROJECT_NAME})

ament_add_gtest(test_data_duration
  data/test_duration.cpp
)
target_link_libraries(test_data_duration ${PROJECT_NAME})

ament_add_gtest(test_data_timezone
  data/test_timezone.cpp
)
target_link_libraries(test_data_timezone ${PROJECT_NAME})

ament_add_gtest(test_data_time
  data/test_time.cpp
)
target_link_libraries(test_data_time ${PROJECT_NAME})

ament_add_gtest(test_data_event
  data/test_event.cpp
)
target_link_libraries(test_data_event ${PROJECT_NAME})

ament_add_gtest(test_data_task
  data/test_task.cpp
)
target_link_libraries(test_data_task ${PROJECT_NAME})

ament_add_gtest(test_data_edge
  data/test_edge.cpp
)
target_link_libraries(test_data_edge ${PROJECT_NAME})

ament_add_gtest(test_data_node
  data/test_node.cpp
)
target_link_libraries(test_data_node ${PROJECT_NAME})

ament_add_gtest(test_data_graph
  data/test_graph.cpp
)
target_link_libraries(test_data_graph ${PROJECT_NAME})

ament_add_gtest(test_data_process
  data/test_process.cpp
)
target_link_libraries(test_data_process ${PROJECT_NAME})

ament_add_gtest(test_data_occurrence
  data/test_occurrence.cpp
)
target_link_libraries(test_data_occurrence ${PROJECT_NAME})

ament_add_gtest(test_data_series
  data/test_series.cpp
)
target_link_libraries(test_data_series ${PROJECT_NAME})

ament_add_gmock(test_data_uuid
  data/test_uuid.cpp
)
target_link_libraries(test_data_uuid ${PROJECT_NAME})

ament_add_gtest(test_data_schedule_change_record
  data/test_schedule_change_record.cpp
)
target_link_libraries(test_data_schedule_change_record ${PROJECT_NAME})


# CACHE
ament_add_gtest(test_cache_action
  cache/test_action.cpp
)
target_link_libraries(test_cache_action ${PROJECT_NAME})

ament_add_gtest(test_cache_event_action
  cache/test_event_action.cpp
)
target_link_libraries(test_cache_event_action ${PROJECT_NAME})

ament_add_gtest(test_cache_task_action
  cache/test_task_action.cpp
)
target_link_libraries(test_cache_task_action ${PROJECT_NAME})

ament_add_gtest(test_cache_process_action
  cache/test_process_action.cpp
)
target_link_libraries(test_cache_process_action ${PROJECT_NAME})

# STORAGE
ament_add_gtest(test_storage_schedule_stream_ld_broker
  storage/test_schedule_stream_ld_broker.cpp
)
target_link_libraries(test_storage_schedule_stream_ld_broker ${PROJECT_NAME})

# UTILS
ament_add_gmock(test_utils_dag_helper
  utils/test_dag_helper.cpp
)
target_link_libraries(test_utils_dag_helper ${PROJECT_NAME})

ament_add_gmock(test_utils_tree_converter
  utils/test_tree_converter.cpp
)
target_link_libraries(test_utils_tree_converter ${PROJECT_NAME})

# HTTP
ament_add_gtest(test_http_memory_stream
  http/test_memory_stream.cpp
)
target_link_libraries(test_http_memory_stream ${PROJECT_NAME})

ament_add_gmock(test_http_connection_curl
  http/test_connection_curl.cpp
)
target_link_libraries(test_http_connection_curl ${PROJECT_NAME})

ament_add_gmock(test_http_transport_curl
  http/test_transport_curl.cpp
)
target_link_libraries(test_http_transport_curl ${PROJECT_NAME})

ament_add_gmock(test_http_request
  http/test_request.cpp
)
target_link_libraries(test_http_request ${PROJECT_NAME})

# ROOT
ament_add_gtest(test_system_time_executor
  test_system_time_executor.cpp
)
target_link_libraries(test_system_time_executor ${PROJECT_NAME})

ament_add_gtest(test_time_window_lookup
  test_time_window_lookup.cpp
)
target_link_libraries(test_time_window_lookup ${PROJECT_NAME})
