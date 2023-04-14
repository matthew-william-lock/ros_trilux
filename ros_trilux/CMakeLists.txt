cmake_minimum_required(VERSION 3.0.2)
project(ros_trilux)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ros_trilux_msgs
)

catkin_package()

###########
## Build ##
###########

# Sources
file(GLOB_RECURSE HDRS include/*.h)
file(GLOB_RECURSE SRCS src/*.cpp)

# Print all sources
message(STATUS "HDRS: ${HDRS}")
message(STATUS "SRCS: ${SRCS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

########################
###### MAKE NODES  #####
########################

# Loop through all the source files
foreach(SRC ${SRCS})
    # Extract the filename without an extension (NAME_WE)
    get_filename_component(TARGET ${SRC} NAME_WE)
    # Add compile target
    add_executable(${TARGET} ${SRC} ${HDRS})
    # Link to Boost libraries AND your targets and dependencies
    target_link_libraries(${TARGET} ${catkin_LIBRARIES})
    add_dependencies(${TARGET} ${catkin_EXPORTED_TARGETS})
endforeach(SRC)