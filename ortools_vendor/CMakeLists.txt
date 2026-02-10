cmake_minimum_required(VERSION 3.5)
project(ortools_vendor)

# find dependencies
find_package(ament_cmake REQUIRED)

if (NOT BUILD_ortools)

  set(PACKAGE_VERSION_MAJOR 9.15)
  set(PACKAGE_VERSION_MINOR 6755)
  set(PACKAGE_UBUNTU_VERSION 22.04)

  set(PACKAGE_VERSION "${PACKAGE_VERSION_MAJOR}.${PACKAGE_VERSION_MINOR}")
  set(PACKAGE_SLUG "or-tools_x86_64_Ubuntu-${PACKAGE_UBUNTU_VERSION}_cpp_v${PACKAGE_VERSION}")
  set(PACKAGE_URL "https://github.com/google/or-tools/releases/download/v${PACKAGE_VERSION_MAJOR}/or-tools_amd64_ubuntu-${PACKAGE_UBUNTU_VERSION}_cpp_v${PACKAGE_VERSION}.tar.gz")
  set(PACKAGE_SHA256 "0b30114d7c05f0596286bf3ef8d02adcf5f45be3b39273490e6bb74a2a9bd1ea")

  # Download OR tools
  file(DOWNLOAD
      ${PACKAGE_URL}
      ${CMAKE_CURRENT_BINARY_DIR}/ortools.tar.gz
      EXPECTED_HASH SHA256=${PACKAGE_SHA256}
      SHOW_PROGRESS
  )

  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ortools")

  add_custom_target(ExtractOrtools ALL
      ${CMAKE_COMMAND} -E
      tar xvzf "${CMAKE_CURRENT_BINARY_DIR}/ortools.tar.gz"
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ortools
  )

  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ortools/${PACKAGE_SLUG}/include/
    DESTINATION include
  )

  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ortools/${PACKAGE_SLUG}/bin/
    DESTINATION bin
  )

  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ortools/${PACKAGE_SLUG}/lib/
    DESTINATION lib
  )

else()
  option(BUILD_absl "" ON)
  option(BUILD_Protobuf "" ON)
  option(BUILD_re2 "" ON)
  option(BUILD_Eigen3 "" OFF)
  option(BUILD_CoinUtils "" ON)
  option(BUILD_Osi "" ON)
  option(BUILD_Clp "" ON)
  option(BUILD_Cgl "" ON)
  option(BUILD_Cbc "" ON)
  option(BUILD_HIGHS "" ON)
  option(BUILD_SCIP "" ON)
  option(BUILD_SAMPLES "" OFF)
  option(BUILD_EXAMPLES "" OFF)
  add_subdirectory(src/or-tools)

  # export lib
  ament_export_libraries(ortools)
  ament_export_include_directories(${CMAKE_INSTALL_INCLUDEDIR})

endif()
ament_package()
