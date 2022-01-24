cmake_minimum_required(VERSION 2.8.3)
project(lslidar_ch_decoder)

add_definitions(-std=c++0x)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  sensor_msgs
  pcl_ros
  pcl_conversions
  lslidar_ch_msgs
  nodelet
)
find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lslidar_ch_decoder
  CATKIN_DEPENDS
    roscpp sensor_msgs pluginlib nodelet
    pcl_ros pcl_conversions
    lslidar_ch_msgs
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

# Lslidar Ch Decoder
add_library(lslidar_ch_decoder
  src/lslidar_ch_decoder.cpp
)
target_link_libraries(lslidar_ch_decoder
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_ch_decoder
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Lslidar Ch Decoder node
add_executable(lslidar_ch_decoder_node
  src/lslidar_ch_decoder_node.cpp
)
target_link_libraries(lslidar_ch_decoder_node
  lslidar_ch_decoder
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_ch_decoder_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Lslidar N301 Decoder nodelet
add_library(lslidar_ch_decoder_nodelet
  src/lslidar_ch_decoder_nodelet.cpp
)
target_link_libraries(lslidar_ch_decoder_nodelet
  lslidar_ch_decoder
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_ch_decoder_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


# install(TARGETS lslidar_ch_decoder_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
