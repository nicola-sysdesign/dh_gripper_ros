cmake_minimum_required(VERSION 2.8.3)
project(dh_gripper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
  serial
  sensor_msgs
  control_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY srv
#   FILES GripperState.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   DIRECTORY action
#   FILES ActuateHand.action
# )

# generate_messages(
#   DEPENDENCIES std_msgs actionlib_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
# LIBRARIES
  CATKIN_DEPENDS roscpp rospy std_msgs actionlib actionlib_msgs serial
    sensor_msgs control_msgs message_runtime
# DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME} SHARED
#include/dh_hand_driver/hand_controller.cpp
#include/dh_hand_driver/DH_datastream.cpp
#include/dh_hand_driver/hand_driver.cpp
#)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(dh_controller src/gripper_node.cpp src/gripper_controller.cpp
src/gripper_driver.cpp src/DH_datastream.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(gripper_controller dh_hand_driver_generate_messages_cpp
#${dh_hand_driver_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(dh_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})


# add_executable(hand_controller_client
# src/test_client.cpp)
# target_link_libraries(hand_controller_client  ${catkin_LIBRARIES})
