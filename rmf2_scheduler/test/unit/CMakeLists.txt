find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)

add_definitions(-DTEST_RESOURCES_DIRECTORY="${TEST_RESOURCES_DIRECTORY}")

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

ament_add_gmock(test_utils_tree_converter
  utils/test_tree_converter.cpp
)
target_link_libraries(test_utils_tree_converter ${PROJECT_NAME})