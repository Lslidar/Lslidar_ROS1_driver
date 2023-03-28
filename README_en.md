## LSLIDAR_CX_ 3.0_ROS

## 1.Introduction

​		LSLIDAR_CX_ 3.0_ROS is the lidar ros driver in linux environment, which is suitable for C16/C32,2.6/2.8/3.0 lidar. The program has  tested under ubuntu18.04 ros melodic , ubuntu 20.04 ros noetic.

## 2.Dependencies

1.ros

To run lidar driver in ROS environment, ROS related libraries need to be installed.

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

### 3.2 Run

run with single  lidar:

~~~bash
roslaunch lslidar_driver lslidar_hs1.launch
~~~

run with double  lidar:

~~~bash
roslaunch lslidar_driver lslidar_hs1_double.launch
~~~



## 4. Introduction to parameters

~~~bash
<launch>
    <arg name="device_ip" default="192.168.1.200" />    #lidar ip
    <arg name="msop_port" default="2368" />   #Main data Stream Output Protocol packet port
    <arg name="difop_port" default="2369" />  #Device Information Output Protocol packet port
    <arg name="lidar_type" default="hs1"/>       # lidar type
    <arg name="pcl_type" default="false"/>        #pointcloud type，false: xyzirt,true:xyzi
    <arg name="use_gps_ts" default="false" />     # Whether gps time synchronization
  <arg name="packet_size" default="1206"/>  #  The udp packet length is 1206 or 1212. If it is 1212 bytes, the radar will be changed to 1212
   <arg name="c16_type" default="c16_2"/>    # c16_ 2 refers to radar with 16-line vertical angle resolution of 2 degrees, c16_ 1 represents radar with 16-line vertical angle resolution of 1.33 degrees
  <arg name="c32_type" default="c32_2"/>   # c32_ 2 refers to the radar with a vertical angle resolution of 1 degree of 32 lines, c32_ 1 represents a radar with a 32-line vertical angle resolution of 0.33 degrees
  <arg name = "c32_fpga_type" default="3"/>  # 3 represents the radar with 32-line fpga of version 2.8/ 3.0, and 2 represents the radar with 32-line fpga of version 2.6
  <node pkg="lslidar_driver" type="lslidar_driver_node" name="lslidar_driver_node" output="screen">
    <!--param name="pcap" value="$(find lslidar_driver)/pcap/tt.pcap"/-->     #Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar
    <param name="device_ip" value="$(arg device_ip)" />
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="pcl_type" value="$(arg pcl_type)"/>
    <param name="lidar_type" value="$(arg lidar_type)"/>
    <param name="packet_size" value="$(arg packet_size)"/>
    <param name="c16_type" value="$(arg c16_type)"/>
    <param name="c32_type" value="$(arg c32_type)"/>
    <param name="c32_fpga_type" value="$(arg c32_fpga_type)"/>
    <param name="add_multicast" value="false"/>    # Whether to add multicast
    <param name="group_ip" value="224.1.1.2"/>       #multicast ip
    <param name="frame_id" value="laser_link"/>     # lidar point cloud coordinate system name
    <param name="min_range" value="0.15"/>         #Unit: m. The minimum value of the lidar blind area, points smaller than this value are filtered
    <param name="max_range" value="1000.0"/>       #Unit: m. The maximum value of the lidar blind area, points smaller than this value are filtered
    <param name="packet_rate" value="840.0"/>      #Number of packets played per second when playing pcap
    <param name="distance_unit" value="0.25"/>   #Radar range resolution
    <param name="angle_disable_min" value="0"/>      #lidar clipping angle start value ，range [0,180]
    <param name="angle_disable_max" value="0"/>      #lidar clipping angle end value ，range [0,180]  
    <param name="horizontal_angle_resolution" value="0.2"/>   #Radar horizontal angle resolution
     <param name="use_gps_ts" value="$(arg use_gps_ts)"/> 
     <param name="scan_num" value="10"/>          #Laserscan line number
    <param name="read_once" value="false"/>      #Whether to play once in pcap offline mode
    <param name="publish_scan" value="false"/>      #Whether to publish the scan
      <param name="pointcloud_topic" value="lslidar_point_cloud"/> #point cloud topic name, can be modified
    <param name="coordinate_opt" value="false"/> //By default, false radar zero-degree angle corresponds to the y-axis of the point cloud, and true radar zero-degree angle corresponds to the x-axis of the point cloud
   
  </node>
</launch>
~~~

### Multicast mode:

- The host computer sets the lidar to enable multicast mode

- Modify the following parameters of the launch file

  ~~~xml
  add_multicast: true
  group_ip: 224.1.1.2    //The multicast ip address set by the host computer
  ~~~

- Run the following command to add the computer to the group (replace enp2s0 in the command with the network card name of the user's computer, you can use ifconfig to view the network card name)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### Offline pcap mode:

- Modify the following parameters of the launch file

  ~~~xml
  // uncomment
      <param name="pcap" value="$(find lslidar_driver)/pcap/tt.pcap"/>   
  #Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar                 
  ~~~



###  pcl point cloud type:

- Modify the following parameters of the launch file

  ~~~xml
  pcl_type: false      # pointcloud type，false: xyzirt,true:xyzi
  ~~~

- The default false is the custom point cloud type, which references the definition in the file of

  lslidar_driver/include/lslidar_driver.h

  Change it to true, which is the own type of pcl:

  ~~~c++
  pcl::PointCloud<pcl::PointXYZI>
  ~~~



## FAQ

Bug Report

Original version : LSLIDAR_CX_V3.2.2_220507_ROS

Modify:  original version

Date    : 2022-05-07



### Updated version:

-----

- LSLIDAR_ CX_ V3.2.3_ 220520_ ROS

- Update description: optimize vertical angle calibration of c32 radar



### Updated version:

---------

- LSLIDAR_ CX_ V3.2.5_ 220729_ ROS

- Update description:

- If the device packet and data packet are not aligned when entering the second, increase the judgment, add 1 second if the second is not entered in time, and subtract 1 second if the second is entered in advance

- Fix the missing bug of c32 radar point cloud

- Do not publish the point cloud before receiving the device package





### Updated version:

------

- LSLIDAR_ CX_ V3.2.6_ 220905_ ROS

- Update description:

- Compatible with 1212-byte version of radar, by modifying the launch file parameter<arg name="packet_size" default="1206"/>//udp package length is 1206 or 1212, if 1212-byte radar is 1212





### Updated version:

--------------

- LSLIDAR_ CX_ V3.2.7_ 221008_ ROS



- Update description:

- Revise the direction of laserscan type topics to keep consistent with pointcloud2 point cloud.



### Updated version:

----------------

- LSLIDAR_ CX_ V3.2.8_ 221214_ ROS

- Update description:

- Optimize the time resolution of 1206-byte radar packet.

- The time between data packets is calculated by subtracting and interpolating two packets.





### Updated version:

----------------

- LSLIDAR_ CX_ V3.2.9_ 221228_ ROS

- Update description:

- Fix the problem of point cloud delamination when the C32 radar compensation angle is negative.



### Updated version:

----------------

- LSLIDAR_ CX_ V3.3.1_ 230313_ ROS

- Update description:

- Reduce cpu usage, boost library to standard library,point time to relative time .
