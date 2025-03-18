cmake_minimum_required(VERSION 3.5)
project(rmf2_scheduler_py)

find_package(ament_cmake REQUIRED)
find_package(rmf2_scheduler REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(pybind11_json_vendor REQUIRED)
find_package(pybind11_json REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

include_directories(
  include
)

ament_python_install_package(rmf2_scheduler)

pybind11_add_module(core
  src/cache/action.cpp
  src/cache/event_action.cpp
  src/cache/process_action.cpp
  src/cache/schedule_cache.cpp
  src/cache/task_action.cpp
  src/core.cpp
  src/data/duration.cpp
  src/data/edge.cpp
  src/data/event.cpp
  src/data/graph.cpp
  src/data/json_serializer.cpp
  src/data/node.cpp
  src/data/process.cpp
  src/data/task.cpp
  src/data/time.cpp
  src/data/time_window.cpp
  src/data/schedule_action.cpp
  src/data/schedule_change_record.cpp
  src/py_utils.cpp
  src/scheduler.cpp
  src/scheduler_options.cpp
  src/storage/schedule_stream.cpp
  src/system_time_executor.cpp
)

target_link_libraries(core
  PRIVATE
  ${rmf2_scheduler_LIBRARIES}
  nlohmann_json::nlohmann_json
  pybind11_json
)

target_include_directories(core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${rmf2_scheduler_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Override flake8 config
  set(ament_cmake_flake8_CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/test/ament_flake8.ini)

  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

install(TARGETS
  core
  DESTINATION "${PYTHON_INSTALL_DIR}/rmf2_scheduler"
)

ament_package()
