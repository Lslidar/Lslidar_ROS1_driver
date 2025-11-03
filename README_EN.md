## Instruction Manual for LSLIDAR_ROS_V5.1.5_250822

### 1.Project Introduction
​		LSLIDAR_ROS_V5.1.5_250822 is a ROS driver for LiDAR in the Linux  environment. The program has been tested and proven compatible with  Ubuntu 20.04 running ROS Noetic, Ubuntu 18.04 with ROS Melodic, and  Ubuntu 16.04 using ROS Kinetic.

#### 1.1  Supported Lidar Models

~~~bash
# Single line Lidars
M10 M10GPS M10P N10 N10Plus N301(1.6 1.7)
~~~

```crystal
# Mechanical Lidars
N301 5.5
C16 C32	# 3.0, 4.0, 5.0 Lidars
C1 C1P C4 C8 C8F CKM8 C16_domestic C32W C32WB C32WN C32WP CH32R CH32RN
MSC16 MS32L
```

~~~elixir
# 905 Hybrid Solid-State Lidars
CX1S3 CX6S3 CH16X1 CH32A CH64W CB64S1_A CX126S3 CH128X1 CH128S1 CX128S2 CH256 CH1W
~~~

```apl
# 1550 Hybrid Solid-State Lidars
LS25D    LS128S1  LS180S1  LS400S1     
LS128S2  LS180S2  LS320S2  LS400S2  MS06
LS128S3  LS144S3  LS180S3  LS320S3  LS400S3
LSS4
```



### 2.Dependencies

#### 2.1 ROS

+ Ubuntu 16.04  -  ROS Kinetic desktop
+ Ubuntu 18.04  -  ROS Melodic desktop
+ Ubuntu 20.04  -  ROS Noetic desktop

#### 2.2 ROS Dependencies 

```bash
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions  ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-message-generation
```

#### 3.2  Other Dependencies

~~~bash
sudo apt-get install libpcap-dev libyaml-cpp-dev libboost${BOOST_VERSION}-dev
~~~



## 3.Compilation and Execution

#### 3.1 Compilation

~~~bash
# Create a workspace and its src directory. lslidar_ws is the workspace name and can be customized.
mkdir -p ~/lidar_ws/src
# Unzip the driver package and place it in the ~/lidar_ws/src directory.
# Return to the workspace.
cd ~/lidar_ws
# Compile the project and refresh the terminal environment.
catkin_make
source devel/setup.bash
~~~

#### 3.2 Execution

To run a single LiDAR:

~~~bash
roslaunch lslidar_driver lslidar_cx.launch # Single line Lidars

roslaunch lslidar_driver lslidar_cx.launch # Mechanical Lidars

roslaunch lslidar_driver lslidar_ch.launch # 905 Hybrid Solid-State Lidars

roslaunch lslidar_driver lslidar_ls.launch # 1550 Hybrid Solid-State Lidars
~~~

To run multiple lidars:

~~~bash
# Customize the launch file according to the actual situation
roslaunch lslidar_driver lslidar_double.launch
~~~



## 4.Parameter Introduction

The content of the **lslidar_cx.launch** file is as follows. More relevant files can be found in the `lslidar_driver/launch` folder.。

For the meaning of each parameter, refer to the comments or consult **`Technical Support`**.。

~~~xml
<launch>
  <arg name="device_ip" default="192.168.1.200"/>   <!-- Lidar IP address -->
  <arg name="msop_port" default="2368"/>            <!-- Lidar destination data port -->
  <arg name="difop_port" default="2369"/>           <!-- Lidar destination device port -->
  <arg name="pcl_type" default="false"/>            <!-- Point cloud type. true: xyzi -->
  <arg name="use_time_service" default="false"/>    <!-- Whether the LiDAR uses time synchronization (GPS, PTP, NTP) -->
  <arg name="use_first_point_time" default="false"/><!-- true: Use the time of the first point in each frame as the point cloud time -->
  <arg name="use_absolute_time" default="false"/> <!-- Point time format. true: Use absolute time. Note: This must be consistent with USE_ABSOLUTE_TIME in the point cloud header file --> 
  <arg name="packet_rate" default="1695.0"/>     <!-- Number of pcap packets read per second, used for offline pcap packet parsing -->

  <node pkg="lslidar_driver" type="lslidar_driver_node" name="lslidar_driver_node" output="screen" ns="cx">
    <!-- Uncomment when parsing PCAP data offline --> 
    <!-- <param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap"/> -->  
    <!-- Custom harness angle cropping file -->
    <param name="filter_angle_file" value="$(find lslidar_driver)/param/filter_angle.yaml"/> 
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="packet_rate" value="$(arg packet_rate)"/>
    <param name="lidar_type" value="CX"/>
    <param name="device_ip" value="$(arg device_ip)"/>
    <param name="msop_port" value="$(arg msop_port)"/>
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="pcl_type" value="$(arg pcl_type)"/>
    <param name="add_multicast" value="false"/>     <!-- Whether to enable multicast. true: multicast mode -->
    <param name="group_ip" value="224.1.1.2"/>      <!-- Lidar multicast IP -->
    <param name="frame_id" value="laser_link"/>     <!-- Point cloud frame ID -->
    <param name="pointcloud_topic" value="lslidar_point_cloud"/>  <!-- Topic name for publishing point cloud data -->
    <param name="use_first_point_time" value="$(arg use_first_point_time)"/> 
    <param name="use_absolute_time" value="$(arg use_absolute_time)"/> 
    <param name="min_range" value="0.15"/>      <!-- Minimum scanning distance of the LiDAR. Points closer than this will be filtered. -->
    <param name="max_range" value="200.0"/>     <!-- Maximum scanning distance of the LiDAR. Points farther than this will be filtered. -->
    <param name="angle_disable_min" value="0"/> <!-- Minimum cropping angle for LiDAR scanning. Enter an integer. Unit: 0.01° -->
    <param name="angle_disable_max" value="0"/> <!-- Maximum cropping angle for LiDAR scanning. Enter an integer. Unit: 0.01° -->
    <param name="horizontal_angle_resolution" value="0.18"/>  <!-- 10Hz: 0.18, 20Hz: 0.36, 5Hz: 0.09 -->
    <param name="publish_scan" value="false"/>   <!-- Whether to publish LaserScan data -->
    <param name="scan_num" value="8"/>           <!-- LaserScan line number -->
    
    <param name="is_pretreatment" value="false"/> <!-- Whether to enable pre - processing. Only applicable to point cloud data -->
    <param name="x_offset" value="0.0"/>  <!-- In RViz, represents the offset around the red x - axis. Unit: m -->
    <param name="y_offset" value="0.0"/>  <!-- In RViz, represents the offset around the green y - axis. Unit: m -->
    <param name="z_offset" value="0.0"/>  <!-- In RViz, represents the offset around the blue z - axis. Unit: m -->
    <param name="roll" value="0.0"/>      <!-- In RViz, represents the rotation around the red x - axis. Unit: rad -->
    <param name="pitch" value="0.0"/>     <!-- In RViz, represents the rotation around the green y - axis. Unit: rad -->
    <param name="yaw" value="0.0"/>       <!-- In RViz, represents the rotation around the blue z - axis. Unit: rad -->
  </node>

  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find lslidar_driver)/rviz/lslidar.rviz"/><!-- Comment out this line to prevent starting RViz -->
 
  <!-- <node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 0 0 0 0 world laser_link 100"/> -->	<!-- TF static coordinate transformation -->
</launch>
~~~

### Explanation of Main Parameters：

- **lidar_type**

  LiDAR types: Single line LiDAR is **`X10`**, mechanical LiDAR is **`CX`**, 905 LiDAR is **`CH`**, and 1550 LiDAR is **`LS`**.

- **lidar_model**

  LiDAR model, specific model of LiDAR, such as M10, CX126S3, LSS3, LSS4, etc.

- **device_ip**

  The IP address of the LiDAR device (note that it is not the destination IP address of the LiDAR). You can check it using `wireshark` or `tcpdump`.

- **msop_port**

  The destination data port of the LiDAR (source port: 2369). Check it with `wireshark` or `tcpdump`.

- **difop_port**

  The destination device port of the LiDAR (source port: 2368). Check it using `wireshark` or `tcpdump`.

- **use_time_service**

  Time synchronization function (ensure the stability of the time source;  otherwise, it will affect the time continuity of the point cloud).

  - true: Use LiDAR time (GPS, NTP, PTP).
  - false: Use system time.

- **use_first_point_time**

  Point cloud time

  - true: Use the timestamp of the first point in each frame as the point cloud time.
  - false: Use the timestamp of the last point in each frame as the point cloud time.

- **use_absolute_time**

  Point time mode (Note: This value must be consistent with **`USE_ABSOLUTE_TIME`** in the **`lslidar_pointcloud.hpp`** file)

  - true: Points use absolute time format.
  - false: Points use relative time format.

- **packet_rate**

  Used when playing `PCAP` data packets offline. It refers to  the number of packets read per second. You can view the number of  packets per second of the LiDAR from the `I/O Chart` in `wireshark` statistics.

- **frame_id**

  The name of the coordinate system for the point cloud data published by  the program. Coordinate system transformation can be achieved through `tf`.

- **pointcloud_topic**

  The topic name for the point cloud data published by the program.

- **min_range  max_range**

  - **`min_range`**: Starting distance of the point cloud.

  - **`max_range`**: Ending distance of the point cloud.

    *This setting is a software filter that will remove points outside this range. Unit: 1m*

- **scan_start_angle  scan_end_angle  angle_disable_min  angle_disable_max**

  - **`scan_start_angle`**: Starting horizontal angle of the point cloud.

    **`scan_end_angle`**: Ending horizontal angle of the point cloud.

    **`angle_disable_min`**: Starting horizontal angle cropping for the point cloud.

    **`angle_disable_max`**: Ending horizontal angle cropping for the point cloud.

    *This setting is a software filter that will remove points outside this angular range. Unit: 0.01°. Enter an integer.*

- **is_pretreatment**

  Point cloud pre - processing. When this value is `true`, the point cloud data will be transformed in spatial position and orientation according to the following parameters.

  - **`x_offset`**: Offset along the x - axis. Unit: m
  - **`y_offset`**: Offset along the y - axis. Unit: m
  - **`z_offset`**: Offset along the z - axis. Unit: m
  - **`roll`**: Rotation around the x - axis. Unit: rad
  - **`pitch`**: Rotation around the y - axis. Unit: rad
  - **`yaw`**: Rotation around the z - axis. Unit: rad

  

### Explanation of Special Parameters：

#### The following functions are only supported by specific series of lidars.

- **serial_port**

  The serial port name connected to the LiDAR (e.g., `/dev/ttyUSB0` or `/dev/ttyACM0`). When using a serial port LiDAR, please fill in the correct serial port  name and ensure that read and write permissions have been granted to the serial port.

- **use_high_precision**

  High-precision mode: When enabled for LaserScan data, the angular  resolution is reduced by a factor of 10, and the data volume increases  10-fold, improving accuracy.

- **publish_multiecholaserscan**
  Has the N10Plus LiDAR been released with `sensor_msgs::MultiEchoLaserScan` data, when this value is true, publish the data.
  
- **enable_noise_filter**
  Isolated noise filtering switch. When this value is true, isolated points will be filtered (not effective for N10Plus).

- **pcl_type**

  Point cloud data type. When this value is `true`, the point cloud data is published in the x, y, z, i format (coordinates and intensity).

- **filter_angle_file**

  - **`disable_min`**: Point cloud data with an angle smaller than this will be filtered out.
  - **`disable_max`**: Point cloud data with an angle smaller than this will be filtered out.

  Custom angle cropping for individual scan lines is allowed. You can set the  cropping angle range for each scan line separately. Modify the **`lslidar_driver/param/filter_angle.yaml`** file according to your actual needs.

- **publish_scan**

  Publish `LaserScan` data. When this value is `true`, `LaserScan` data will be published.

- **scan_num**

  Specify which scan line is used to publish `LaserScan` data. The valid line numbers depend on the number of scan lines of the LiDAR.

- **echo_mode**

  Echo mode. 0: Publish all point clouds 1: Publish the first echo point cloud 2: Publish the second echo point cloud(Effective in dual echo mode).

- **is_add_frame**

  Stacked-frame point cloud publishing: The consecutive two frames of point cloud data are stacked and published together.

- **packet_loss**

  Packet loss detection. Once enabled, the driver will publish the total number of LiDAR packet losses as a topic named **`packet_loss`**, with the message type **`std_msgs::Int64`**.

  

​		

### Multicast Mode:

- Set the destination IP of the LiDAR to a multicast network segment.

- Modify the following parameters in the launch file:

  ~~~shell
  <param name="add_multicast" value="false"/>    # Whether to enable multicast
  <param name="group_ip" value="224.1.1.2"/>     # Multicast IP address
  ~~~

- Run the following commands to add the computer to the multicast group (replace `enp2s0` with the network card name of your computer, which can be checked using `ifconfig`):

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### Offline PCAP Playback Mode:

- Copy the recorded `PCAP` file to the **`lslidar_driver/pcap`** folder.

- Modify the following parameters in the launch file.

  ~~~shell
  # Path of the pcap package. Uncomment this when loading the pcap package.
  <param name="pcap" value="$(find lslidar_cx_driver)/pcap/xxx.pcap" />   
  ~~~



### PCL Point Cloud Type：

- The driver publishes point clouds in a custom point type, defined in **`lslidar_driver/include/lslidar_pointcloud.hpp`**.

- To use absolute time, uncomment **`#define USE_ABSOLUTE_TIME`**.

  ~~~c++
  struct PointXYZIRT {
      PCL_ADD_POINT4D;      // x, y, z, and data[4]
      PCL_ADD_INTENSITY;    // intensity  
      std::uint16_t ring;   // laser ring ID  
      POINT_TIME_TYPE time; // time (POINT_TIME_TYPE -> float/double)  
  
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
  ~~~



### Device Information:

- The LiDAR driver publishes the device information in the form of a topic. The topic name is `lslidar_device_info`, and the message type is `lslidar_msgs::LslidarInformation`. For the definition, refer to **`lslidar_msgs/LslidarInformation.msg`**.

  

### Fault Codes：

- The LS (1550) series LiDAR driver publishes the fault codes in the form of a topic. The topic name is `lslidar_fault_code`, and the message type is `std_msgs::String`.

  *For the meaning of each fault code, please contact the `technical support`.*



### Point Cloud Time：

- The LiDAR driver publishes the point cloud time in the form of a topic. The topic name is `time_topic`, and the message type is `std_msgs::Float64`.





## 5.Lidar Function Settings and Control

#### **Some functions are only supported by specific series of lidars.**



### Setting the Lidar Network Configuration：

~~~bash
# Open a new terminal, cx is the namespace.
source devel/setup.bash

# Configure the LiDAR using explicit parameters.
rosservice call /cx/network_setup "lidar_ip: '192.168.1.200'
destination_ip: '192.168.1.102'
data_port: 2368
dev_port: 2369" 

# Configure the LiDAR using the parameters in the configuration file (network_setup.yaml).
rosservice call /cx/network_setup "lidar_ip: ''
destination_ip: ''
data_port: 0
dev_port: 0" 
~~~

#### ***Restart the driver after making changes.***





### Lidar Timing Mode：

~~~bash
# Open a new terminal, cx is the namespace.
source devel/setup.bash

# 0: GPS   1: PTP L2   2: NTP   3: PTP UDPv4   4: E2E L2   5: E2E UDPv4
rosservice call /cx/time_mode "time_mode: 0
ntp_ip: ''" 

# When setting NTP timing, you need to fill in the NTP IP.
rosservice call /cx/time_mode "time_mode: 2
ntp_ip: '192.168.1.102'"
~~~





### Lidar Power On/Off (Mechanical):

~~~bash
# Open a new terminal, cx is the namespace.
source devel/setup.bash
~~~

Power On：

~~~bash
rosservice call /cx/power_control "power_control: 1"
~~~

Power Off：

~~~bash
rosservice call /cx/power_control "power_control: 0"
~~~





### Lidar Rotation/Stop Rotation (Mechanical):

~~~bash
# Open a new terminal, cx is the namespace.
source devel/setup.bash
~~~

Rotation：

~~~bash
rosservice call /cx/motor_control "motor_control: 1"
~~~

Stop Rotation：

~~~bash
rosservice call /cx/motor_control "motor_control: 0"
~~~





### Lidar Rotation Speed (Mechanical, 905 Series):

~~~bash
# Open a new terminal, cx is the namespace.
source devel/setup.bash
# Optional frequency  5Hz/10Hz/20Hz
rosservice call /cx/motor_speed "motor_speed: 20"	# 5 10 20 (Extensions 30, 40, 50, 60 are only settable for CX1S3)
~~~





### Rain, Fog, and Dust Removal Level of the Lidar(Mechanical)：

~~~bash
# Open a new terminal, cx is the namespace.
source devel/setup.bash
#Available levels: 0/1/2/3. The higher the number, the stronger the removal effect.
rosservice call /cx/remove_rain_fog_dust "rfd_removal: 3"
~~~





### Tail Removal Level of the Lidar(Mechanical)：

~~~bash
# Open a new terminal, cx is the namespace.
source devel/setup.bash
# Available levels range from 0-10. The higher the number, the stronger the removal effect.
# The maximum level for older LiDAR models is 4.
rosservice call /cx/tail_remove "tail_removal: 4"	
~~~





### Angle Distortion Correction of the Lidar(LSS3)：

**The LSS4 series radar currently does not support this feature. Calling this service will configure whether to send invalid data.**

~~~bash
# Open a new terminal, ls is the namespace.
source devel/setup.bash
~~~

Turn off angle distortion correction:：

~~~bash
rosservice call /ls/angle_distortion_correction "angle_distortion_correction: 0" 
~~~

Turn on angle distortion correction:：

~~~bash
rosservice call /ls/angle_distortion_correction "angle_distortion_correction: 1" 
~~~

**Enabling this function can reduce CPU usage**

#### ***Restart the driver after making changes.***





### Lidar Frame Rate(1550)：

~~~bash
# Open a new terminal, ls is the namespace.
source devel/setup.bash
# 0: Normal frame rate;  1: 50% frame rate;  2: 25% frame rate
rosservice call /ls/frame_rate "frame_rate: 1"		# 50% frame rate, 5Hz
~~~





### Sending of Invalid Lidar Data(1550)：

**For the LSS4 series radar, please call the `angle_distortion_correction` service to use this feature.**

~~~bash
# Open a new terminal. ls is the namespace.
source devel/setup.bash
~~~

Send invalid data:

~~~bash
rosservice call /ls/invalid_data "invalid_data: 0"
~~~

Do not send invalid data：

~~~bash
rosservice call /ls/invalid_data "invalid_data: 1"
~~~

***Not sending invalid data can reduce CPU usage, but it will cause discontinuity in point - cloud time.***





### Lidar Standby Mode(1550)：

~~~bash
# Open a new terminal, ls is the namespace.
source devel/setup.bash
~~~

Normal mode：

~~~bash
rosservice call /ls/standby_mode "standby_mode: 0"
~~~

Standby mode：

~~~bash
rosservice call /ls/standby_mode "standby_mode: 1"
~~~





### Single-line Lidar Motor Control:

~~~bash
# N301 is currently not supported
# Open a new terminal, x10 as the namespace
source devel/setup.bash
~~~

Rotation:

~~~bash
rostopic pub -1 /x10/motor_control std_msgs/Int8 "data: 1"
~~~

Stop rotation and cease data transmission:

~~~bash
rostopic pub -1 /x10/motor_control std_msgs/Int8 "data: 0"
~~~









## FAQ

Bug Report

Original version : LSLIDAR_ROS_V5.0.8_250210

Modify: original version

Date    : 2025-02-10

--------------------------------------------------------------------



update version : LSLIDAR_ROS_V5.0.9_250224

Modify: 

1. Added compatibility with the CH32A LiDAR.
2. Added the publication of the point - cloud time topic.
3. Update domestic C16 vertical angle.

Date    : 2025-02-24

--------------------------------------------------------------------



update version : LSLIDAR_ROS_V5.1.0_250324

Modify: 

1. Added compatibility with the M10, M10GPS, M10P, N10, N301 LiDAR.
1. Added single-line LiDAR motor control functionality.
1. Added compatibility with 1550 LSS4 series LiDAR.

Date    : 2025-03-24

--------------------------------------------------------------------



update version : LSLIDAR_ROS_V5.1.1_250403

Modify: 

1. Added compatibility with N10Plus LiDAR.
1. N10Plus LiDAR adds MultiEchoLaserScan message.
1. Add Laserscan high-precision mode.
1. Add point cloud time and point time format selection.

Date    : 2025-04-03

--------------------------------------------------------------------



update version : LSLIDAR_ROS_V5.1.2_250427

Modify: 

1. Added compatibility with the MS32L LiDAR.
1. Update the vertical angle of the mechanical C32 5.0 LiDAR.

Date    : 2025-04-27

--------------------------------------------------------------------

update version : LSLIDAR_ROS_V5.1.2_250507

Modify: 

1. Added git management.

Date    : 2025-05-07

------

update version : LSLIDAR_ROS_V5.1.3_250704

Modify: 

1. Correcting the issue of incorrect timing for mechanical v4.0 PTP.

Date    : 2025-07-04

--------------------------------------------------------------------

update version : LSLIDAR_ROS_V5.1.4_250724

Modify: 

1. Enhance compatibility with CH1W Lidar Type

Date    : 2025-07-24

------

update version : LSLIDAR_ROS_V5.1.5_250822

Modify: 

1. Extended speed setting (extended to 30, 40, 50, 60, only available for CX1S3)

Date    : 2025-08-22
