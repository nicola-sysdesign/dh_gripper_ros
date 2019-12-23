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
  hardware_interface
  controller_manager
  serial
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate services in the 'srv' folder
add_service_files(
  DIRECTORY srv
  FILES hand_state.srv
)
## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES ActuateHand.action
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

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
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    actionlib
    actionlib_msgs
    hardware_interface
    controller_manager
    serial
    message_runtime
  DEPENDS
    Boost
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

add_executable(dh_hardware_interface src/dh_hardware_interface.cpp)

target_link_libraries(dh_hardware_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(hand_controller_client
src/test_client.cpp)
target_link_libraries(hand_controller_client  ${catkin_LIBRARIES})
