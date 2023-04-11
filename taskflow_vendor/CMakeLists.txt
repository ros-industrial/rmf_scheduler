cmake_minimum_required(VERSION 3.18)

project(taskflow_vendor)

option(FORCE_BUILD_VENDOR_PKG
  "Build taskflow from source, even if system-installed package is available"
  OFF)

find_package(ament_cmake REQUIRED)
macro(build_taskflow)
  set(extra_cmake_args)

  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -w")

  list(APPEND extra_cmake_args "-DTF_BUILD_TESTS=OFF")
  list(APPEND extra_cmake_args "-DTF_BUILD_EXAMPLES=OFF")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}")
  list(APPEND extra_cmake_args "-DCMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")

  # TODO(anyone): Verify Windows build
  if(WIN32 AND NOT ${CMAKE_VERBOSE_MAKEFILE})
    set(should_log ON)  # prevent warnings in Windows CI
  else()
    set(should_log OFF)
  endif()

  if(DEFINED CMAKE_TOOLCHAIN_FILE)
    list(APPEND extra_cmake_args "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}")
  endif()

  # Cuda flag
  if(DEFINED USE_CUDA)
    # Add additional cuda related cmake variables
    list(APPEND extra_cmake_args "-DTF_BUILD_CUDA=ON")
  endif()

  include(ExternalProject)
  ExternalProject_Add(Taskflow
    GIT_REPOSITORY https://github.com/taskflow/taskflow
    GIT_TAG v3.5.0
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/taskflow_install
      ${extra_cmake_args}
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/taskflow_install/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/taskflow_vendor
    USE_SOURCE_PERMISSIONS
  )
endmacro()

if(NOT FORCE_BUILD_VENDOR_PKG)
  find_package(Taskflow QUIET)
endif()

if(NOT Taskflow_FOUND OR FORCE_BUILD_VENDOR_PKG)
  build_taskflow()
else()
  message(STATUS "Found taskflow ${Taskflow_VERSION} in path ${Taskflow_CONFIG}")
endif()

ament_package(
  CONFIG_EXTRAS "taskflow_vendor-extras.cmake.in"
)
