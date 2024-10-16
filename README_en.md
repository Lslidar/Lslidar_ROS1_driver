# LSLIDAR_LS1500_ROS

## 1.Introduction
​		LSLIDAR_LS1550_ROS  is the lidar ros driver in linux environment, which is suitable for   LS500   lidar. The program has  tested under ubuntu 20.04 ros noetic and ubuntu18.04 ros melodic.

## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

**Ubuntu 16.04**: ros-kinetic-desktop-full

**Ubuntu 18.04**: ros-melodic-desktop-full

**Ubuntu 20.04**: ros-noetic-desktop-full

**Installation**: please refer to [http://wiki.ros.org](http://wiki.ros.org/)

2.ros dependencies

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions 
```

3.other dependencies

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #Select the appropriate version
~~~

## 3.Compile & Run

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
roslaunch lslidar_driver lslidar_ls500.launch
~~~

run with double lidar:

~~~bash
roslaunch lslidar_driver lslidar_ls500_double.launch
~~~

## 4. Introduction to parameters

The content of the lslidar_ls128.launch file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
<launch>
  <arg name="device_ip" default="192.168.1.200" />     #lidar ip
  <arg name="msop_port" default="2368" />              # Main data Stream Output Protocol packet port
  <arg name="difop_port" default="2369" />             # Device Information Output Protocol packet port
  <arg name="use_time_service" default="false" />  # Whether use time service

    <node pkg="lslidar_driver" type="lslidar_driver_node" name="lslidar_driver_node" output="screen">
<!--    <param name="pcap" value="$(find lslidar_ls128_driver)/pcap/xxx.pcap"/>-->
    <!--<param name="pcap" value="/home/ls/Desktop/ls128s1/wireshark.pcap"/>-->   #Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
    <param name="lidar_ip" value="$(arg device_ip)"/>
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="add_multicast" value="false"/>              # Whether to add multicast
    <param name="group_ip" value="224.1.1.2"/>               #multicast ip
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="frame_id" value="laser_link"/>              # lidar point cloud coordinate system name
    <param name="is_add_frame_" value="false"/>				 # Whether stacke frame
    <param name="min_range" value="0.15"/>                   # Minimum measuring distance of lidar
    <param name="max_range" value="500.0"/>                  # Maximum measuring distance of lidar
    <param name="scan_start_angle" value="-60"/>             #Scan start angle, range - 60 ° to 60 °
    <param name="scan_end_angle" value="60"/>                #Scan end angle,   range - 60 ° to 60 °
    <param name="pointcloud_topic" value="lslidar_point_cloud"/>  # name of point cloud topic
    <param name="Vertical_angle_compensation" value="-28.0"/> # Vertical angle compensation
    <param name="time_synchronization" value="$(arg time_synchronization)"/>  # Whether gps time synchronization
    <param name="publish_laserscan" value="false"/>           #Whether to publish the scan

  <!--node name="rviz" pkg="rviz" type="rviz" args="-d $(find lslidar_driver)/launch/lslidar_ls.rviz" output="screen"/-->

</launch>
~~~

## 5. Packet loss detection

Drive to publish the total number of lidar packet loss in the form of topic with the name of "packet_loss" and the message type of std_ msgs::msg::Int64。


## FAQ

Bug Report

Original version : LSLIDAR_LS500_v1.0.1_231219_ROS

Modify:  original version

Date    : 2023-12-19




Original version : LSLIDAR_LS500_v1.0.2_240308_ROS

Modify:  Add vertical angle compensation

Date    : 2024-03-08