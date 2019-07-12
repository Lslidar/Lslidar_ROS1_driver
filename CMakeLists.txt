cmake_minimum_required(VERSION 2.8.3)
project(ls01b_v2)

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  message_generation
        dynamic_reconfigure
)
find_package(Boost REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/Filter.cfg
)

add_service_files(FILES resolution.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_srvs
rospy std_msgs message_runtime dynamic_reconfigure)

include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


add_executable(ls01b_v2
                src/lsiosr.cpp
                src/ls01b.cpp)
add_dependencies(ls01b_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ls01b_v2 ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ls01b_v2
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
