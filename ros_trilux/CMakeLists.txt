cmake_minimum_required(VERSION 3.0.2)
project(ros_trilux)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ros_trilux_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread serialization)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_trilux
)

###########
## Build ##
###########

# Sources
file(GLOB_RECURSE HDRS include/*.h)
file(GLOB_RECURSE SRCS src/*.cpp)

# Print all sources
# message(STATUS "HDRS: ${HDRS}")
# message(STATUS "SRCS: ${SRCS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(ros_trilux
  src/trilux_node.cpp
  include/trilux/trilux_serial.h
)

target_link_libraries(ros_trilux
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_THREAD_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
)

## Declare a cpp executable
add_executable(trilux_node src/trilux_node.cpp include/trilux/trilux_node.h include/trilux/trilux_serial.h)

target_link_libraries(trilux_node
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${Boost_THREAD_LIBRARY}
  ${Boost_SERIALIZATION_LIBRARY}
)