cmake_minimum_required(VERSION 2.8.3)
project(lslidar_msgs)

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
        LslidarC16Sweep.msg
        LslidarC32Sweep.msg
        LslidarScanUnified.msg
)


add_service_files(
        FILES
        lslidar_control.srv
        motor_control.srv
        motor_speed.srv
        time_service.srv
        data_port.srv
        dev_port.srv
        data_ip.srv
        destination_ip.srv
)


generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
        CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
