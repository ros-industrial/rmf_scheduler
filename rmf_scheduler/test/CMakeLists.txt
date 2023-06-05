set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions( -DTEST_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

ament_add_gtest(test_${PROJECT_NAME}
  ./test_dag.cpp
  ./test_events_handler.cpp
  ./test_series.cpp
  ./test_system_time_executor.cpp
  ./test_scheduler.cpp
)
target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  gvc
  cgraph
)
