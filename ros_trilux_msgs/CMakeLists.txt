cmake_minimum_required(VERSION 3.0.2)
project(ros_trilux_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
    FILES 
    Measurement.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  EnableAnalog.srv
  EnableReporting.srv
  EnableSingleShot.srv
  GetMeasurement.srv
  Reboot.srv
  SaveSetup.srv
  SetRate.srv
  StopStartMeasurements.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

## Catkin specific configuration
catkin_package()