cmake_minimum_required(VERSION 3.0.2)
project(dh_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  actionlib
  actionlib_msgs
  serial
  sensor_msgs
  control_msgs
  industrial_msgs
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
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    actionlib
    actionlib_msgs
    serial
    sensor_msgs
    control_msgs
    industrial_msgs
  DEPENDS Boost
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
add_executable(dh_controller
src/gripper_node.cpp src/gripper_controller.cpp src/dh_driver.cpp src/DH_datastream.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(gripper_controller dh_hand_driver_generate_messages_cpp
#${dh_hand_driver_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(dh_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
install(
  TARGETS dh_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES rosconsole.conf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
