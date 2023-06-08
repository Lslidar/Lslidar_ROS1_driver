### LSLIDAR_CH64 Driver Description



### Initial Version:

* LSLIDAR_CH64_V1.0.1_210914_ROS

### Updated Version:

- LSLIDAR_CH64_V2.0.1_221216_ROS
- Updates：combine decoder node and  driver node into one driver node to reduce the CPU usage.

### Range of Applications：

* Applicable to CH64

### Compile and Run

~~~shell
mkdir -p ~/lslidar_ws/src
cd ~/lslidar_ws/src
Copy the whole driver package to a workspace such as ~/lslidar_ws, and then unzip it
cd ~/lslidar_ws
catkin_make
source devel/setup.bash
roslaunch lslidar_driver lslidar_ch64.launch   
~~~



### Parameters Description in the  Launch File:

~~~xml
<launch>
  <arg name="device_ip" default="192.168.1.200" />  #Lidar IP
  <arg name="msop_port" default="2368"/>   #Destination port of data package 
  <arg name="difop_port" default="2369"/>   #Destination port of device package
   <arg name="lidar_type" default="ch64"/>  #Lidar Ch64
  <arg name="pcl_type" default="false" />   /#Point cloud type,false:xyzirt; true: xyzi
 <arg name="use_time_service" default="false" />  #Whether use GPS timing or PTP timing service.

    
   
   <node pkg="lslidar_driver" type="lslidar_ch_driver_node" name="lslidar_driver_node" output="screen">
    <!-- param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap"/-->#Uncomment（delet!-- --）,enable offline pcap mode
    <param name="lidar_ip" value="$(arg device_ip)"/>
     <param name="lidar_type" value="$(arg lidar_type)"/>
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="add_multicast" value="false"/>  #Whether enable multicast mode.
    <param name="group_ip" value="224.1.1.2"/> #Multicast IP address
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="frame_id" value="laser_link"/> #Coordinates ID
    <param name="pcl_type" value="$(arg pcl_type)"/>    #Point cloud type, false:xyzirt; true:xyzi
    <param name="min_range" value="0.15"/>  #Unit:m; minimum value of lidar's blind area; points smaller than this value are filtered.
    <param name="max_range" value="150.0"/> /#Unit: m; maximum value of lidar's blind area, pints bigger than this value are filtered.
    <param name="angle_disable_min" value="0"/>  #Start value of lidar's clipping value, unit: 1°.
    <param name="angle_disable_max" value="0"/>  #End value of lidar's clipping value, unit: 1°.
    <Param name="pointcloud_topic" value="lslidar_point_cloud"/>  #name of poind cloud topic, which can be modified.
    <param name="horizontal_angle_resolution" value="0.09"/>  #lidar's angular resolution
    <param name="packet_rate" value="3571.0"/>  #data package sent each second
    <param name="channel_num" value="16"/>  #laserscan channel number
    <param name="echo_num" value="0"/>  #Only valid in dual echo mode, 0 means publish all points cloud, 1 means publish the point cloud of 1st echo and 2 means publish point cloud of 2nd echo.
    <param name="publish_laserscan" value="false"/> #Whether publish laserscan topic.
  </node>
 
 <!--node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 1 0 0 0 world laser_link 100" /-->  #Uncomment（delet!-- --）,static coordinates conversion


~~~

### Multicast Mode:

- Enable multicast mode on Windows Client

- Modify parameters below in launch file

  ~~~xml
      <param name="add_multicast" value="true"/> 
      <param name="group_ip" value="224.1.1.2"/>  #Multicast IP address set on Windows Client
  ~~~

- Run the command below to add computer to the group(replace enp2s0 with network card name of user's computer,which can be viewed using ifconfig command )

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~





### Offline Pcap Mode：

- Copy the recorded pcap file and paste it to file folder *lslidar_driver/pcap*

- Modify the parameters in launch file

  ~~~xml
     //Uncomment
   	 <param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap">  # 
  ~~~



###  Point Cloud Type

- Modify the parameters below in launch file

  ~~~xml
    <arg name="pcl_type" default="false" />   #point cloud type,false:xyzirt; true:xyzi
  ~~~

  

- By default, false is customized point  cloud type, for definition, refer to lslidar_driver/include/lslidar_ch_driver/lslidar_ch_driver.h

- Change it to true, which is the own type of pcl

  ~~~c++
  pcl::PointCloud<pcl::PointXYZI>
  ~~~





