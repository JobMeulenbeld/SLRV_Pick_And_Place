﻿################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(slrv_pick_and_place)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    open_manipulator_msgs
    ar_track_alvar_msgs
    message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  Swap.msg
  Status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    open_manipulator_msgs
    ar_track_alvar_msgs
    message_runtime 
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(slrv_pick_and_place src/slrv_pick_and_place.cpp)
add_dependencies(slrv_pick_and_place ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(slrv_pick_and_place ${catkin_LIBRARIES} )

################################################################################
# Install
################################################################################
install(TARGETS slrv_pick_and_place
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
