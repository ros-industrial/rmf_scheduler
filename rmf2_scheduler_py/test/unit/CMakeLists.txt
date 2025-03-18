find_package(ament_cmake_pytest REQUIRED)

add_definitions(-DTEST_RESOURCES_DIRECTORY="${TEST_RESOURCES_DIRECTORY}")

set(PYTEST_ENV
  AMENT_PREFIX_INDEX=${ament_index_build_path}
  PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
)
set(LD_PRELOAD "")

# Special flags for address sanitizer
if(CMAKE_CXX_FLAGS MATCHES "-fsanitize=address")
  if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(
      COMMAND gcc -print-file-name=libasan.so
      OUTPUT_VARIABLE asan_RUNTIME_LIBS
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    message(STATUS "Found GCC Runtime Address Sanitizer library in ${asan_RUNTIME_LIBS}!")
    list(APPEND LD_PRELOAD "${asan_RUNTIME_LIBS}")
  elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    execute_process(
      COMMAND uname -m
      COMMAND tr -d '\n'
      OUTPUT_VARIABLE ARCHITECTURE
    )
    execute_process(
      COMMAND clang -print-file-name=libclang_rt.asan-${ARCHITECTURE}.so
      OUTPUT_VARIABLE asan_RUNTIME_LIBS
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    message(STATUS "Found Clang Runtime Address Sanitizer library in ${asan_RUNTIME_LIBS}!")
    list(APPEND LD_PRELOAD ${asan_RUNTIME_LIBS})
  endif()

  list(APPEND PYTEST_ENV
    ASAN_OPTIONS=detect_leaks=0  # Disable leak detection
  )
endif()

# Special flags for address sanitizer
if(CMAKE_CXX_FLAGS MATCHES "-fsanitize=thread")
  if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(
      COMMAND gcc -print-file-name=libtsan.so
      OUTPUT_VARIABLE tsan_RUNTIME_LIBS
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    message(STATUS "Found GCC Runtime Thread Sanitizer library in ${tsan_RUNTIME_LIBS}!")
    list(APPEND LD_PRELOAD "${tsan_RUNTIME_LIBS}")
  endif()
endif()

# Setup LD_PRELOAD for sanitizers
if(NOT LD_PRELOAD STREQUAL "")
  list(APPEND PYTEST_ENV
    LD_PRELOAD=${LD_PRELOAD}
  )
endif()

message(STATUS "PYTEST_ENV ${PYTEST_ENV}")

# DATA
ament_add_pytest_test(test_data_time
  data/test_time.py
  APPEND_ENV ${PYTEST_ENV}
)

ament_add_pytest_test(test_data_event
  data/test_event.py
  APPEND_ENV ${PYTEST_ENV}
)

ament_add_pytest_test(test_data_process
  data/test_process.py
  APPEND_ENV ${PYTEST_ENV}
)

# CACHE
ament_add_pytest_test(test_cache_task_action
  cache/test_task_action.py
  APPEND_ENV ${PYTEST_ENV}
)
