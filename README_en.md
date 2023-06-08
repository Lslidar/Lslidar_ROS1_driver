### LSLIDAR_CH120 Driver Description



### Original Version:

* LSLIDAR_CH120_V1.0.5_220110_ROS

### Updated Version:

- LSLIDAR_CH120_V2.0.1_230222_ROS
- Updates:  Calculation formula for vertical angle is updated; node combination.

### Updated version：

- LSLIDAR_CH120_V2.0.3_230302_ROS
- Updates: Fix the bug of abnormal display of scan topic.

### Range of Application

* Applicable to CH120

### Compile&Run：

~~~shell
mkdir -p ~/lslidar_ws/src
cd ~/lslidar_ws/src
Copy the whole lidar ROS driver directory into ROS workspace, i.e "~/lidar_ws/src".
cd ~/lslidar_ws
catkin_make
source devel/setup.bash
roslaunch lslidar_driver lslidar_ch120.launch   
~~~



### Parameters Description:

~~~xml
<launch>
  <arg name="device_ip" default="192.168.1.200" />  #Lidar IP
  <arg name="msop_port" default="2368"/>   #Destination port of data package
  <arg name="difop_port" default="2369"/>   #Destination port of device package
   <arg name="lidar_type" default="ch120"/>  #Ch120
  <arg name="pcl_type" default="false" />   #Point cloud type, false:xyzirt; true:xyzi.
 <arg name="use_time_service" default="false" />  #Whether lidar use GPS timing or PTP timing.
    
   
   <node pkg="lslidar_driver" type="lslidar_ch_driver_node" name="lslidar_driver_node" output="screen">
    <!-- param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap"/-->//Uncomment（delet!-- --），enable offline pcap mode
    <param name="lidar_ip" value="$(arg device_ip)"/>
     <param name="lidar_type" value="$(arg lidar_type)"/>
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="add_multicast" value="false"/>  #Whether enable multicast mode or not
    <param name="group_ip" value="224.1.1.2"/>  #Multicast IP address
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="frame_id" value="laser_link"/>  #ID of coordinates
    <param name="pcl_type" value="$(arg pcl_type)"/>    #Point cloud type, false:xyzirt; true:xyzi.
    <param name="min_range" value="0.15"/> #Unit: m. Minimum value of lidar's blind area, points smaller than this value are filtered.
    <param name="max_range" value="150.0"/> #Unit: m. Manimum value of lidar's blind area, points bigger than this value are filtered.
    <param name="angle_disable_min" value="0"/> #Start value of lidar's clipping angle, unit: 1°.
    <param name="angle_disable_max" value="0"/>  #End value of lidars' clipping angle, unit:1°.
    <Param name="pointcloud_topic" value="lslidar_point_cloud"/>  #Name name of point cloud topic, which can be modified.
    <param name="horizontal_angle_resolution" value="0.36"/>  //Horizontal angular resolution. 10Hz: 0.36°, 20Hz: 0.72°, 5Hz:0.18°.
    <param name="packet_rate" value="2525.0"/>  #Number of package played each second
    <param name="channel_num" value="60"/>  #Channel number of laserscan
    <param name="echo_num" value="0"/>  #Only valid, 1 means publish cloud of the 1st echo and 2 means publish poind cloud of the 2nd echo.
    <param name="publish_laserscan" value="false"/> #Whether publish laser scan topic.
  </node>
 
 <!--node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 1 0 0 0 world laser_link 100" /-->  #Ubcomment（delet!-- --）, static coordinates conversion


~~~

### Multicast Mode：

- Enable multicast mode on Windows Client

- Modify the parameters below in launch file

  ~~~xml
      <param name="add_multicast" value="true"/> 
      <param name="group_ip" value="224.1.1.2"/>   #Multicast IP address set on Windows Client
  ~~~

- Run the following command to add the computer to the group(replace the "enp2s0" with the network card name of user's computer, which can be viewed with ifconfig command)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~





### Offline pcap mode:

- Copy the recorded pcap file and paste it to file folder *lslidar_driver/pcap*

- Modify the parameters of launch file

  ~~~xml
     #Uncomment
   	 <param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap">  # chnage xxx.pcap to the copied pcap file name
  ~~~



###  pcl Point Cloud Type：

- Modify the parameters below in launch file

  ~~~xml
    <arg name="pcl_type" default="false" />   #Point cloud type, false:xyzirt; true:xyzi.
  ~~~

  

- By default, false is the custom point cloud type, for its definition, refer to:

  lslidar_driver/include/lslidar_ch_driver/lslidar_ch_driver.h

- Change it to true, which is the own type of pcl

  ~~~c++
  pcl::PointCloud<pcl::PointXYZI>
  ~~~





