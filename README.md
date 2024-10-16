# LSLIDAR_LS1550_ROS使用说明

## 1.工程介绍
​		LSLIDAR_LS1550_ROS为linux环境下雷达ros驱动，适用于 LS500系列雷达，程序在ubuntu 20.04 ros noetic和ubuntu18.04 ros melodic下测试通过。

## 2.依赖

1.ubuntu20.04 ros noetic/ubuntu18.04 ros melodic/ubuntu16.04 ros kinetic

2.ros依赖

```bash
# 安装
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pluginlib  ros-$ROS_DISTRO-pcl-conversions
```

3.其他依赖

pcap,boost

~~~bash
sudo apt-get install libpcap-dev
sudo apt-get install libboost${BOOST_VERSION}-dev   #选择适合的版本
~~~



## 3.编译运行

### 3.1 编译

~~~bash
mkdir -p ~/lidar_ws/src
#将ROS压缩包解压缩放到~/lidar_ws/src 目录下
cd ~/lidar_ws
catkin_make
source devel/setup.bash
~~~

### 3.2 运行

运行单个雷达:

~~~bash
roslaunch lslidar_driver lslidar_ls500.launch
~~~

运行多个雷达：

~~~bash
roslaunch lslidar_driver lslidar_ls500_double.launch
~~~

## 4.参数介绍

lslidar_ls1550.launch文件内容如下，每个参数含义见注释说明。

~~~bash
<launch>
  <arg name="device_ip" default="192.168.1.200" />     #雷达ip
  <arg name="msop_port" default="2368" />              #数据包端口
  <arg name="difop_port" default="2369" />             # 设备包端口
  <arg name="use_time_service" default="false" />  #是否授时

    <node pkg="lslidar_driver" type="lslidar_driver_node" name="lslidar_driver_node" output="screen">
    <!--<param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap"/>-->
    <param name="lidar_ip" value="$(arg device_ip)"/>
    <param name="msop_port" value="$(arg msop_port)" />
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="add_multicast" value="false"/>              #是否添加组播
    <param name="group_ip" value="224.1.1.2"/>               #组播的ip
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="frame_id" value="laser_link"/>              # 雷达点云坐标系
    <param name="is_add_frame_" value="false"/>				       # 是否叠帧
    <param name="min_range" value="0.15"/>                   # 雷达的最小测量距离
    <param name="max_range" value="200.0"/>                  # 雷达的最大测量距离
    <param name="pointcloud_topic" value="lslidar_point_cloud"/>  # 发布雷达点云话题名称
    <param name="Vertical_angle_compensation" value="-28.0"/> # 垂直角度补偿
    <param name="scan_start_angle" value="-60"/>              #扫描起始角度，范围-60°到60°
    <param name="scan_end_angle" value="60"/>                 #扫描结束角度，范围-60°到60°
     <param name="time_synchronization" value="$(arg time_synchronization)"/>  #是否gps时间同步 
    <param name="publish_laserscan" value="false"/>           #是否发布scan
 
  </node>

  <!--node name="rviz" pkg="rviz" type="rviz" args="-d $(find lslidar_driver)/launch/lslidar_ls.rviz" output="screen"/-->

</launch>
~~~

## 5.丢包检测

驱动将雷达丢包总数以topic的形式发布出来，topic名字为"packet_loss"，消息类型为std_msgs::msg::Int64。


## FAQ

Bug Report

Original version : LSLIDAR_LS500_v1.0.1_231219_ROS

Modify:  original version

Date    : 2023-12-19




Original version : LSLIDAR_LS500_v1.0.2_240308_ROS

Modify:  新增launch中修改垂直角度补偿

Date    : 2024-03-08