cmake_minimum_required(VERSION 2.8.3)
project(lslidar_driver)


add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(libpcap_LIBRARIES -lpcap)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        lslidar_msgs
        pluginlib
        sensor_msgs
        pcl_ros
        pcl_conversions
        )

find_package(Boost COMPONENTS signals)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
        include
        ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES lslidar_ch_driver
        CATKIN_DEPENDS
        roscpp  
        lslidar_msgs
        sensor_msgs
        pluginlib
        pcl_ros pcl_conversions
        DEPENDS
        Boost
)



link_directories(
        ${catkin_LIBRARY_DIRS}
)

add_library(lslidar_input
        src/input.cc
        )

target_link_libraries(lslidar_input
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
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

# Leishen ch lidar node
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


if (catkin_EXPORTED_TARGETS)
    add_dependencies(lslidar_input ${catkin_EXPORTED_TARGETS})
endif ()




# install options
# install(TARGETS lslidar_ch_driver_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY launch
#         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

