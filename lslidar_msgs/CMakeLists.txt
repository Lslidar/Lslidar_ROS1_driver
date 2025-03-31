cmake_minimum_required(VERSION 2.8.3)
project(lslidar_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  LslidarInformation.msg
  LslidarPacket.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  AngleDistortionCorrection.srv
  FrameRate.srv
  InvalidData.srv
  IpAndPort.srv
  MotorControl.srv
  MotorSpeed.srv
  PowerControl.srv
  RfdRemoval.srv
  StandbyMode.srv
  TailRemoval.srv
  TimeMode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lslidar_msgs
 CATKIN_DEPENDS message_generation std_msgs
#  DEPENDS system_lib
)