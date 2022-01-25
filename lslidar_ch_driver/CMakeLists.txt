cmake_minimum_required(VERSION 2.8.3)
project(lslidar_ch_driver)

add_definitions(-std=c++0x)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_updater
  nodelet

  lslidar_ch_msgs
)

find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    #  LIBRARIES lslidar_ch_driver
    CATKIN_DEPENDS
    roscpp diagnostic_updater nodelet
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

add_library(lslidar_input
  src/input.cc
)

target_link_libraries(lslidar_input
  ${catkin_LIBRARIES}
)

# Leishen ch lidar driver
add_library(lslidar_ch_driver
  src/lslidar_ch_driver.cc
)

target_link_libraries(lslidar_ch_driver
  lslidar_input
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_ch_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Leishen ch lidar nodelet
add_executable(lslidar_ch_driver_node
  src/lslidar_ch_driver_node.cc
)

target_link_libraries(lslidar_ch_driver_node
  lslidar_ch_driver
  ${catkin_LIBRARIES}
)
add_dependencies(lslidar_ch_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Leishen ch lidar nodelet
add_library(lslidar_ch_driver_nodelet
  src/lslidar_ch_driver_nodelet.cc
)

target_link_libraries(lslidar_ch_driver_nodelet
  lslidar_input
  ${catkin_LIBRARIES}
)

target_link_libraries(lslidar_ch_driver_nodelet
  lslidar_ch_driver
  ${catkin_LIBRARIES}
)

if(catkin_EXPORTED_TARGETS)
        add_dependencies(lslidar_input ${catkin_EXPORTED_TARGETS})
endif()

add_dependencies(lslidar_ch_driver_nodelet
  lslidar_input
  lslidar_ch_driver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


# install options
# install(TARGETS lslidar_ch_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

