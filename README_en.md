# LSLIDAR_N301_V3.0.0_241017_ROS illustrate



## 1.Introduction

​		LSLIDAR_N301_V3.0.0_241017_ROS  is a lidar ROS driver in a Linux environment. The program has been tested successfully under ubuntu 20.04 ROS nonetic, ubuntu18.04 ROS media, and ubuntu16.04 ROS kinetic.

Applicable to the LeiShen N301 LiDAR for Protocols 1.6 and 1.7.

## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

**Ubuntu 16.04**: ros-kinetic-desktop-full

**Ubuntu 18.04**: ros-melodic-desktop-full

**Ubuntu 20.04**: ros-noetic-desktop-full

**Installation**: please refer to [http://wiki.ros.org]

2.ros dependencies

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pluginlib ros-$ROS_DISTRO-diagnostic-updater
```

3.other dependencies

~~~bash
sudo apt-get install libpcap-dev 
~~~

## 3.Compile && Run

### 3.1 Compile

~~~bash
mkdir -p ~/lidar_ws/src
~~~

Copy the whole lidar ROS driver directory into ROS workspace, i.e "~/lidar_ws/src".

~~~bash
cd ~/lidar_ws
catkin_make
source devel/setup.bash
~~~

3.2 Run

run with single lidar:

~~~bash
roslaunch lslidar_n301_driver lslidar_n301.launch
~~~

run with double lidar:

~~~bash
roslaunch lslidar_n301_driver lslidar_double.launch 
~~~



## 4. Introduction to parameters

The content of the lslidar_n301.launch file is as follows, and the meaning of each parameter is shown in the notes.

~~~xml
<launch>
  <arg name="device_ip" default="192.168.1.200" />   <!-- LiDAR IP -->
  <arg name="msop_port" default="2368"/>             <!-- Port for LiDAR data -->
  <arg name="difop_port" default="2369"/>            <!-- Port for LiDAR device -->
  <arg name="use_time_service" default="false" />    <!-- Indicates whether the LiDAR uses time service (GPS) -->
  <arg name="packet_rate" default="840.0"/>

  <node pkg="lslidar_n301_driver" type="lslidar_n301_driver_node" name="lslidar_driver_node" output="screen">
    <!--param name="pcap" value="$(find lslidar_n301_driver)/pcap/xxx.pcap" /-->
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="packet_rate" value="$(arg packet_rate)"/>
    <param name="device_ip" value="$(arg device_ip)" />
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="add_multicast" value="false"/>     <!-- Enable multicast mode: true to use multicast -->
    <param name="group_ip" value="224.1.1.2"/>      <!-- Multicast IP -->
    <param name="protocol" value="1"/>              <!-- 1: Protocol 1.7, 2: Protocol 1.6 -->
    <param name="frame_id" value="laser_link"/>     <!-- Point cloud frame ID -->
    <param name="laserscan_topic" value="scan"/>    <!-- laserscan topic -->
    <param name="distance_min" value="0.15"/>       <!-- Minimum scanning distance; points below this value will be filtered -->
    <param name="distance_max" value="200.0"/>      <!-- Maximum scanning distance; points above this value will be filtered -->
    <param name="angle_disable_min" value="12000"/>     <!-- Minimum clipping angle for scanning, unit: 0.01° -->
    <param name="angle_disable_max" value="24000"/>     <!-- Maximum clipping angle for scanning, unit: 0.01° -->
    <param name="horizontal_angle_resolution" value="0.18"/>  <!-- Horizontal angle resolution: 5Hz:0.09, 10Hz:0.18, 20Hz:0.36 -->
    <param name="coordinate_opt" value="true"/>    <!-- Direction corresponding to 0-degree angle in point cloud; true: positive x-axis -->
  </node>

  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find lslidar_n301_driver)/rviz/lslidar.rviz"/> <!-- Comment this line if RViz is not needed -->
  
</launch>

~~~

- ### Multicast mode:

  - The host computer sets the lidar to enable multicast mode

  - Modify the following parameters of the launch file

  ~~~shell
  <param name="add_multicast" value="true"/>                 # add multicast
  <param name="group_ip" value="224.1.1.2"/>                 # The multicast ip address set by the host computer
  ~~~

- Run the following command to add the computer to the group (replace enp2s0 in the command with the network card name of the user's computer, you can use ifconfig to view the network card name)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



- ### Offline pcap mode:

  - Copy the recorded pcap file to the lslidar_n301_driver/pcap folder
  
  - Modify the following parameters of the launch file
  
  ~~~shell
  // uncomment
      <param name="pcap" value="$(find lslidar_n301_driver)/pcap/xxx.pcap" /> 
  ~~~

- ### 


## FAQ

Bug Report

Original version : LSLIDAR_N301_V3.0.0_241017_ROS

Modify:  original version

Date    : 2024-10-17

--------------------------------------------------------------------



