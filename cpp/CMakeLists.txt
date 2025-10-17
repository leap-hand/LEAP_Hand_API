cmake_minimum_required(VERSION 3.14)

project(
  LEAP_HAND_API
  LANGUAGES CXX C
  VERSION 1.0.0)

set(PLATEFORM "linux64" CACHE STRING "Plateform to build dynamixel sdk")
message(STATUS "Please set your plateform with -DPLATEFORM={linux_sbc, linux32, linux64, mac} default: linux64")

find_package(Eigen3 REQUIRED)

## Build dynamixel sdk
add_custom_target(
    run_make_dynamixel_sdk ALL
    COMMAND make && sudo make install
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/dynamixel_sdk/build/${PLATEFORM}
    COMMENT "Running dynamixel_sdk Makefile"
)

set(SRC_CLIENT
  src/dynamixel_client.cpp
  src/leap_hand_utils.cpp
  src/leap_controller.cpp
)

## Build dynamixel client library
add_library(dynamixel_client SHARED ${SRC_CLIENT})
add_dependencies(dynamixel_client run_make_dynamixel_sdk)
target_link_libraries(dynamixel_client PUBLIC
  dxl_x64_cpp
  Eigen3::Eigen
)
target_include_directories(dynamixel_client PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

# Build test_leap_hand
add_executable(test_leap_hand src/main.cpp)
target_link_libraries(test_leap_hand PRIVATE dynamixel_client)