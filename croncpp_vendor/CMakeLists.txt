cmake_minimum_required(VERSION 3.18)

project(croncpp_vendor)

option(FORCE_BUILD_VENDOR_PKG
  "Build croncpp from source, even if system-installed package is available"
  OFF)

find_package(ament_cmake REQUIRED)
macro(build_croncpp)
  set(extra_cmake_args)

  if(DEFINED CMAKE_BUILD_TYPE)
    list(APPEND extra_cmake_args -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE})
  endif()

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -w")

  list(APPEND extra_cmake_args "-DCRONCPP_BUILD_TESTS=OFF")
  list(APPEND extra_cmake_args "-DCRONCPP_BUILD_BENCHMARK=OFF")
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

  include(ExternalProject)
  ExternalProject_Add(croncpp
    GIT_REPOSITORY https://github.com/mariusbancila/croncpp
    GIT_TAG v2023.03.30
    CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_BINARY_DIR}/croncpp_install
      ${extra_cmake_args}
  )

  # The external project will install to the build folder, but we'll install that on make install.
  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/croncpp_install/include/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/croncpp_vendor/include/croncpp
    USE_SOURCE_PERMISSIONS
  )

  install(
    DIRECTORY
      ${CMAKE_CURRENT_BINARY_DIR}/croncpp_install/lib/
    DESTINATION
      ${CMAKE_INSTALL_PREFIX}/opt/croncpp_vendor/lib
    USE_SOURCE_PERMISSIONS
  )
endmacro()

if(NOT FORCE_BUILD_VENDOR_PKG)
  find_package(croncpp QUIET)
endif()

if(NOT croncpp_FOUND OR FORCE_BUILD_VENDOR_PKG)
  build_croncpp()
else()
  message(STATUS "Found croncpp ${croncpp_VERSION} in path ${croncpp_CONFIG}")
endif()

ament_package(
  CONFIG_EXTRAS "croncpp_vendor-extras.cmake.in"
)
