set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions( -DTEST_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Event data structure test
ament_add_gtest(test_${PROJECT_NAME}_data
  ./test_dag.cpp
  ./test_events_handler.cpp
  ./test_series.cpp
  ./test_event_details_filter.cpp
)

target_link_libraries(test_${PROJECT_NAME}_data
  ${PROJECT_NAME}
  gvc
  cgraph
)

# System time executor test
ament_add_gtest(test_${PROJECT_NAME}_runtime
  ./test_system_time_executor.cpp
  ./test_dag_executor.cpp
)
target_link_libraries(test_${PROJECT_NAME}_runtime
  ${PROJECT_NAME}
)

# Scheduler static event API testing
ament_add_gtest(test_${PROJECT_NAME}_static_event_api
  ./test_scheduler.cpp
)

target_link_libraries(test_${PROJECT_NAME}_static_event_api
  ${PROJECT_NAME}
)

# Conflict checking
ament_add_gtest(test_${PROJECT_NAME}_conflict_identifier
  ./test_identifier.cpp
)

target_link_libraries(test_${PROJECT_NAME}_conflict_identifier
  ${PROJECT_NAME}
)

# CP Solver
ament_add_gtest(test_${PROJECT_NAME}_cp_solver
  ./test_cp_solver.cpp
)

target_link_libraries(test_${PROJECT_NAME}_cp_solver
  ${PROJECT_NAME}
)
