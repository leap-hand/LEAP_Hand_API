cmake_minimum_required(VERSION 3.5)
project(leap_ros2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder

#############
## Install ##
#############

# Install Python scripts
install(PROGRAMS
  scripts/leaphand_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all Python scripts in the scripts directory
file(GLOB SCRIPT_FILES scripts/*.py)
install(PROGRAMS ${SCRIPT_FILES}
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

###################################
## ament specific configuration  ##
###################################
ament_package()

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# find_package(ament_cmake_gtest REQUIRED)
# ament_add_gtest(${PROJECT_NAME}-test test/test_leap_hand.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
