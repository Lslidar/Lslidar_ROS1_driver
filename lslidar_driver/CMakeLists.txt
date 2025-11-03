cmake_minimum_required(VERSION 2.8.3)
project(lslidar_driver)

add_definitions(-std=c++14)

set(libpcap_LIBRARIES -lpcap)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    pcl_conversions
    std_msgs
    sensor_msgs
    dynamic_reconfigure
    message_generation
    lslidar_msgs
)

find_package(Boost COMPONENTS signals)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(yaml-cpp REQUIRED)

set(yaml-cpp_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIRS})
set(yaml-cpp_LIBRARIES ${YAML_CPP_LIBRARIES})

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
   roscpp 
   pcl_ros 
   pcl_conversions 
   std_msgs 
   sensor_msgs 
   lslidar_msgs
 DEPENDS 
   Boost 
   yaml-cpp
)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")

add_library(lslidar_driver
  src/input.cpp
  src/lsiosr.cpp
  src/lslidar_ch_driver.cpp
  src/lslidar_cx_driver.cpp
  src/lslidar_ls_driver.cpp
  src/lslidar_x10_driver.cpp
  src/lslidar_services.cpp
  src/lslidar_device_info.cpp
)
add_dependencies(lslidar_driver ${catkin_EXPORTED_TARGETS})
target_link_libraries(lslidar_driver
  ${YAML_CPP_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(lslidar_driver_node src/lslidar_driver_node.cpp)
add_dependencies(lslidar_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lslidar_driver_node
  lslidar_driver
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS lslidar_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS lslidar_driver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  FILES_MATCHING PATTERN "*.yaml"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
  PATTERN ".svn" EXCLUDE
)