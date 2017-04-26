cmake_minimum_required(VERSION 2.8.3)
project(orb_slam2_ros)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  orb_slam2_msgs
)

find_package(OpenCV 2.4 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES orb_slam2_ros
  CATKIN_DEPENDS roscpp std_msgs orb_slam2_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(orb_slam2_talker test/test_talker.cpp)
add_executable(orb_slam2_listener test/test_listener.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(orb_slam2_talker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(orb_slam2_listener
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  #PATTERN ".svn" EXCLUDE
)