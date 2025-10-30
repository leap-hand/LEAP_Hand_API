cmake_minimum_required(VERSION 3.14)

project(
  LEAP_HAND_API
  LANGUAGES CXX C
  VERSION 1.0.0
)

# Platform selection
set(PLATEFORM "linux64" CACHE STRING "Platform to build dynamixel sdk (linux_sbc, linux32, linux64, mac)")
message(STATUS "Building for platform: ${PLATEFORM}")

# Dependencies
find_package(Eigen3 REQUIRED)

# === Build Dynamixel SDK via Makefile ===
add_custom_target(
    run_make_dynamixel_sdk ALL
    COMMAND make && sudo make install
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/dynamixel_sdk/build/${PLATEFORM}
    COMMENT "Building and installing Dynamixel SDK"
)

# === Library sources ===
set(SRC_CLIENT
  src/dynamixel_client.cpp
  src/leap_hand_utils.cpp
  src/leap_controller.cpp
)

# === Include directories ===
set(PUBLIC_HEADERS
  include/leap_hand_utils/dynamixel_client.h
  include/leap_hand_utils/leap_hand_utils.h
  include/leap_hand_utils/leap_controller.h
)

# === Shared library ===
add_library(dynamixel_client SHARED ${SRC_CLIENT} ${PUBLIC_HEADERS})
add_dependencies(dynamixel_client run_make_dynamixel_sdk)

target_include_directories(dynamixel_client
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(dynamixel_client
  PUBLIC
    dxl_x64_cpp
    Eigen3::Eigen
)

# === Executable ===
add_executable(test_leap_hand src/main.cpp)
target_link_libraries(test_leap_hand PRIVATE dynamixel_client)

# === Install targets ===
include(GNUInstallDirs)

install(
  TARGETS dynamixel_client
  EXPORT leap_hand_clientTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/leap_hand_utils DESTINATION include)

# Export the library for use with find_package
install(EXPORT leap_hand_clientTargets
  FILE leap_hand_clientTargets.cmake
  NAMESPACE leap_hand_client::
  DESTINATION lib/cmake/leap_hand_client
)

# === Config file for find_package ===
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/leap_hand_clientConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/leap_hand_clientConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/leap_hand_clientConfig.cmake"
  INSTALL_DESTINATION lib/cmake/leap_hand_client
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/leap_hand_clientConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/leap_hand_clientConfigVersion.cmake"
  DESTINATION lib/cmake/leap_hand_client
)