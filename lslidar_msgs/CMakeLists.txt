cmake_minimum_required(VERSION 2.8.3)
project(lslidar_msgs)

set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  LslidarPacket.msg
  LslidarPoint.msg
  LslidarScan.msg
)


add_service_files(
  FILES
  lslidar_control.srv
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
