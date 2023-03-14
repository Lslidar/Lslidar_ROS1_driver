# LSLIDAR_CH1W_V1.0.4_220913_ROS

## 1.Introduction
​		LSLIDAR_CH1W_V1.0.4_220913_ROS is the lidar ros driver in linux environment, which is suitable for ch1w lidar. The program has  tested under ubuntu 20.04 ros noetic and ubuntu18.04 ros melodic.

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
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions  ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-nodelet
```

3.other dependencies

~~~bash
sudo apt-get install libpcap-dev
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

### 3.2 Run

run with single lidar:

~~~bash
roslaunch lslidar_ch1w_decoder lslidar_ch1w.launch
~~~

run with double lidar:

~~~bash
roslaunch lslidar_ch1w_decoder lslidar_ch_double.launch
~~~

## 4. Introduction to parameters

The content of the lslidar_ch1w.launch file is as follows, and the meaning of each parameter is shown in the notes.

~~~bash
<launch>
  <arg name="device_ip" default="192.168.1.200" />  #lidar ip
  <arg name="msop_port" default="2368"/>   # Main data Stream Output Protocol packet port
  <arg name="difop_port" default="2369"/>   # Device Information Output Protocol packet port
   
  <arg name="pcl_type" default="false" />   #pointcloud type，false: xyzirt,true:xyzi
 <arg name="use_time_service" default="false" />  # Whether gps time synchronization
    
    
  <node pkg="lslidar_ch1w_driver" type="lslidar_ch1w_driver_node" name="lslidar_ch1w_driver_node" output="screen">
    <!-- param name="pcap" value="$(find lslidar_ch1w_driver)/pcap/xxx.pcap"/-->#Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
    <param name="lidar_ip" value="$(arg device_ip)"/>
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="read_once" value="false"/>    //Whether to read the pcap packet only once
    <param name="read_fast" value="false"/>    //Whether to speed up reading pcap packets
    <param name="repeat_delay" value="0.0"/>   // Pause time between repeated reading of pcap packets
    <param name="add_multicast" value="false"/> # Whether to add multicast
    <param name="group_ip" value="224.1.1.2"/>  #multicast ip
    <param name="use_time_service" value="$(arg use_time_service)"/>
  </node>
    
      <node pkg="lslidar_ch1w_decoder" type="lslidar_ch1w_decoder_node" name="lslidar_ch1w_decoder_node" output="screen">
    <param name="frame_id" value="laser_link"/> # lidar point cloud coordinate system name
    <param name="pcl_type" value="$(arg pcl_type)"/>   
    <param name="min_range" value="0.15"/> # Unit: m. The minimum value of the lidar blind area, points smaller than this value are filtered
    <param name="max_range" value="150.0"/> # Unit: m. The maximum value of the lidar blind area, points smaller than this value are filtered
    <param name="angle_disable_min" value="0"/> #lidar clipping angle start value, unit 1°
    <param name="angle_disable_max" value="0"/> #lidar clipping angle end value, unit 1°
    <Param name="pointcloud_topic" value="lslidar_point_cloud"/>  #point cloud topic name, can be modified
    <param name="frequency" value="10"/>  # lidar frequency
    <param name="pulse_repetition_rate" value="60.0"/> 
    <param name="publish_point_cloud" value="true"/> #Whether to publish the point cloud
    <param name="publish_laserscan" value="false"/>  # Whether to publish the scan
  </node>
 
 <!--node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 1 0 0 0 world laser_link 100" /-->  #Uncomment (delete!-- --), static coordinate system transformation

~~~

### Multicast mode:

- The host computer sets the lidar to enable multicast mode

- Modify the following parameters of the launch file

  ~~~xml
      <param name="add_multicast" value="true"/> 
      <param name="group_ip" value="224.1.1.2"/>  //The multicast ip address set by the host computer
  ~~~

- Run the following command to add the computer to the group (replace enp2s0 in the command with the network card name of the user's computer, you can use ifconfig to view the network card name)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### Offline pcap mode:

- Copy the recorded pcap file to the lslidar_ch1w/lslidar_ch1w_driver/pcap folder

- Modify the following parameters of the launch file

  ~~~xml
     // uncomment
   	 <param name="pcap" value="$(find lslidar_ch1w_driver)/pcap/xxx.pcap">  // xxx.pcap is changed to the copied pcap file name
  ~~~



###  pcl point cloud type:

- Modify the following parameters of the launch file

  ~~~xml
    <arg name="pcl_type" default="false" />   #pointcloud type，false: xyzirt,true:xyzi
  ~~~

- The default false is the custom point cloud type, which defines the reference

  lslidar_ch1w/lslidar_ch1w_decoder/include/lslidar_ch_decoder/lslidar_ch_decoder.h

  Change it to true, which is the own type of pcl:

  ~~~c++
  pcl::PointCloud<pcl::PointXYZI>
  ~~~

## FAQ

Bug Report

version : LSLIDAR_CH1W_V1.0.2_220623_ROS

Modify:  original version

Date    : 2022-06-23

-------------------------------------------------------------------------------------

version : LSLIDAR_CH1W_V1.0.3_220704_ROS

Modify:  1. fix bug

Date    : 2022-07-24

----------------------

version : LSLIDAR_CH1W_V1.0.4_220913_ROS

Modify:  1.Add dual echo mode

Date    : 2022-09-13
