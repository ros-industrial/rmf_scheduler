set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions( -DTEST_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}\")

ament_add_gtest(test_${PROJECT_NAME}
  ./test_identifier.cpp
  ./test_resolver.cpp
)

target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
)
