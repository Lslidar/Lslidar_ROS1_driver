# Lslidar_M10_N10_V2.5.3_231110_ROS Instruction

## 1.Introduction

​		Lslidar_M10_N10_V2.5.3_231110_ROS is the lidar ros driver in linux environment, which is suitable for M10 ,M10_GPS,M10_P,M10_PLUS,N10,N10_PLUS and L10 lidar. The program has  tested under ubuntu 20.04 ros noetic , ubuntu18.04 ros melodic and ubuntu16.04 ros kinetic.



## 2.Dependencies

- #### ros

​	To run lidar driver in ROS environment, ROS related libraries need to be installed.

​	**Ubuntu 16.04**: ros-kinetic-desktop-full

​	**Ubuntu 18.04**: ros-melodic-desktop-full

​	**Ubuntu 20.04**: ros-noetic-desktop-full

​	**Installation**: please refer to [http://wiki.ros.org]



- #### ros dependencies

```bash
# install
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions  ros-$ROS_DISTRO-diagnostic-updater
```



- #### other dependencies

~~~bash
sudo apt-get install libpcap-dev 
sudo apt-get install libboost${BOOST_VERSION}-dev   #Select the appropriate version
~~~



## 3.Compile && Run

- This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH` after cloning the package to your workspace. And the normal procedure for compling a catkin package will work.

```bash
mkdir -p ~/lidar_ws/src
#Copy the whole lidar ROS driver directory into ROS workspace, i.e "~/lidar_ws/src".
cd ~/lidar_ws
catkin_make
source devel/setup.bash

#run
roslaunch lslidar_driver lslidar_net.launch 				#net lidar
roslaunch lslidar_driver lslidar_serial.launch 				#serial lidar
```



## 4. Introduction to parameters

- The content of the lslidar_serial.launch file is as follows, and the meaning of each parameter is shown in the notes.

#### When using serial port radar, serial port permissions should be granted

~~~bash
sudo chmod 777 /dev/ttyUSB0								#ttyUSB0 is lidar serial port
~~~



~~~xml
<launch>
  <node pkg="lslidar_driver" type="lslidar_driver_node" name="lslidar_driver_node" output="screen">
    <param name="lidar_name" value="M10"/>     #lidar type:M10 M10_P M10_PLUS M10_GPS N10 N10_PLUS L10
    <param name="serial_port" value="/dev/ttyUSB0"/>      #lidar connection serial port
    <param name="interface_selection" value="serial"/>    #interface select: net or serial
    <param name="frame_id" value="laser_link"/>           #lidar coordinates 
    <param name="scan_topic" value="scan"/>               #Topic name of lidar scan
    <param name="angle_disable_min" value="0.0"/>         #Crop angle start
    <param name="angle_disable_max" value="0.0"/>         #Crop angle end
    <param name="min_range" value="0.0"/>                 #lidar receiving distance min
    <param name="max_range" value="100.0"/>               #lidar receiving distance max
    <param name="use_gps_ts" value="false"/>              #lidar use GPS timing
    <param name="compensation" value="false"/>            #Using angle compensation
    <param name="pubScan" value="true"/>                  #Lidar Release Scanning Theme
    <param name="pubPointCloud2" value="false"/>          #Lidar Release pointcloud2 Theme
    <param name="high_reflection" value="false"/>         #M10_P lidar This value needs to be filled in,Uncertain, please contact technical support.
    <!--param name="in_file_name" value="$(find lslidar_driver)/pcap/xxx.txt"/-->   #Using the txt file reading function
  </node>
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find lslidar_driver)/rviz/lslidar.rviz" output="screen"/>      #Visualization
</launch>
~~~



- The content of the lslidar_net.launch file is as follows, and the meaning of each parameter is shown in the notes.

~~~xml
<launch>
  <node pkg="lslidar_driver" type="lslidar_driver_node" name="lslidar_driver_node" output="screen">
    <param name="frame_id" value="laser_link"/>          #lidar scan/point cloud coordinate system name
    <param name="device_ip" value="192.168.1.200"/>      #lidar ip
    <param name="device_port" value="2368"/>             #Main data Stream Output Protocol packet port
    <param name="device_ip_difop" value="192.168.1.102"/>#lidar destination IP
    <param name="difop_port" value="2369"/>              #Device Information Output Protocol packet port
    <param name="lidar_name" value="M10"/>               #lidar type:M10 M10_P M10_PLUS M10_GPS N10 N10_PLUS L10M10_GPS N10 N10_PLUS L10
    <param name="interface_selection" value="net"/>      #interface select: net or serial
    <param name="add_multicast" value="false"/>          #Whether to add multicast
    <param name="group_ip" value="224.1.1.2"/>           #multicast ip
    <param name="scan_topic" value="scan"/>              #set lidar data topic name
    <param name="angle_disable_min" value="0.0"/>        #Crop angle start
    <param name="angle_disable_max" value="0.0"/>        #Crop angle end
    <param name="min_range" value="0"/>                  #lidar receiving distance min
    <param name="max_range" value="100.0"/>              #lidar receiving distance max
    <param name="use_gps_ts" value="false"/>             #lidar use GPS timing
    <param name="compensation" value="false"/>           #Using angle compensation
    <param name="pubScan" value="true"/>                 #Lidar Release Scanning Theme
    <param name="pubPointCloud2" value="false"/>         #Lidar Release pointcloud2 Theme
    <param name="high_reflection" value="false"/>        #M10_P lidar This value needs to be filled in,Uncertain, please contact technical support.
    <!--param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap"/-->  #Using the pcap file reading function
  </node>
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find lslidar_driver)/rviz/lslidar.rviz" output="screen"/>     #Visualization
</launch>
~~~



## 5. Command controlled radar

- Run lidar drive,open new terminal

~~~bash
#lidar type: M10 M10_GPS M10_PLUS N10 N10_PLUS 
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 0"		#close radar
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 1"		#open radar

#lidar type: M10 M10_GPS M10_P
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 2"		#no filtering
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 3"		#normal filtering
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 4"		#close range filtering
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 100"    #Lidar sends device packets

#Only M10 Start stop version
rostopic pub -1 /lslidar_order std_msgs/Int8 "data: 30"		#close radar & stop send data
~~~





#### Offline pcap mode:

- Modify the following parameters of the launch file

  ~~~bash
  #uncomment
      <param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap"/> 
  #Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar 
  ~~~

  

#### Offline txt mode:

- Modify the following parameters of the launch file

~~~bash
#uncomment
    <param name="in_file_name" value="$(find lslidar_driver)/pcap/xxx.txt"/> 
#Uncomment to read the data from the pcap file, and add the comment to read the data from the lidar 
~~~

