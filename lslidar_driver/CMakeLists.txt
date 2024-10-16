cmake_minimum_required(VERSION 2.8.3)
project(lslidar_driver)

add_definitions(-std=c++14)
set(libpcap_LIBRARIES -lpcap)

#FIND_PACKAGE( OpenMP REQUIRED)
#if(OPENMP_FOUND)
#    message("OPENMP FOUND")
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()

set(CMAKE_BUILD_TYPE Release)
message("cmake_build_type:"  ${CMAKE_BUILD_TYPE})

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        pcl_conversions
        nodelet
        lslidar_msgs
        )

#find_package(Boost REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES lslidar_ls_driver
        CATKIN_DEPENDS
        roscpp  nodelet
        lslidar_msgs  pcl_ros pcl_conversions
        DEPENDS
        #    Boost
)

include_directories(
        include
        #  ${Boost_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
)

link_directories(
        ${catkin_LIBRARY_DIRS}
)

add_library(lslidar_input_ls128
        src/input.cc
        )

target_link_libraries(lslidar_input_ls128
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES}
        )

# Leishen ch lidar driver
add_library(lslidar_128_driver
        src/lslidar_ls_driver.cc
        )

target_link_libraries(lslidar_128_driver
        lslidar_input_ls128
        ${catkin_LIBRARIES}
        )

add_dependencies(lslidar_128_driver
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

# Leishen ch lidar nodelet
add_executable(lslidar_driver_node
        src/lslidar_ls_driver_node.cc
        )

target_link_libraries(lslidar_driver_node
        lslidar_128_driver
        ${catkin_LIBRARIES}
        )
add_dependencies(lslidar_driver_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

if (catkin_EXPORTED_TARGETS)
    add_dependencies(lslidar_input_ls128 ${catkin_EXPORTED_TARGETS})
endif ()


# install options
install(TARGETS lslidar_input_ls128 lslidar_128_driver lslidar_driver_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
