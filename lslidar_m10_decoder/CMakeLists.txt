cmake_minimum_required(VERSION 2.8.3)
project(lslidar_m10_decoder)

add_compile_options(-std=c++14)
set( CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  sensor_msgs
  pcl_ros
  pcl_conversions
  lslidar_m10_msgs
)
find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lslidar_m10_decoder
  CATKIN_DEPENDS
    roscpp sensor_msgs pluginlib
    pcl_ros pcl_conversions
    lslidar_m10_msgs
  DEPENDS
    Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

# Lslidar M10 Decoder
add_library(lslidar_m10_decoder
  src/lslidar_m10_decoder.cpp
)
target_link_libraries(lslidar_m10_decoder
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_m10_decoder
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Lslidar M10 Decoder node
add_executable(lslidar_m10_decoder_node
  src/lslidar_m10_decoder_node.cpp
)
target_link_libraries(lslidar_m10_decoder_node
  lslidar_m10_decoder
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_m10_decoder_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS lslidar_m10_decoder_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
