# LSLIDAR_ROS_V5.1.5_250822 使用说明

## 1.工程介绍
​		LSLIDAR_ROS_V5.1.5_250822为linux环境下雷达ros驱动，程序在ubuntu 20.04 ros noetic,ubuntu18.04 ros melodic以及ubuntu16.04 ros kinetic下测试通过。

#### 1.1 支持的雷达型号

~~~bash
# 单线激光雷达
M10 M10GPS M10P N10	N10Plus N301(1.6 1.7)
~~~

```crystal
# 机械式激光雷达
N301 5.5
C16 C32	#3.0 4.0 5.0雷达
C1 C1P C4 C8 C8F CKM8 C16_domestic C32W C32WB C32WN C32WP CH32R CH32RN
MSC16 MS32L(5.0)
```

~~~elixir
# 905混合固态激光雷达
CX1S3 CX6S3 CH16X1 CH32A CH64W CB64S1_A CX126S3 CH128X1 CH128S1 CX128S2 CH256 CH1W
~~~

```apl
# 1550混合固态激光雷达
LS25D    LS128S1  LS180S1  LS400S1     
LS128S2  LS180S2  LS320S2  LS400S2  MS06
LS128S3  LS144S3  LS180S3  LS320S3  LS400S3
LSS4
```



### 2.依赖

#### 2.1 ROS

+ Ubuntu 16.04  -  ROS Kinetic desktop
+ Ubuntu 18.04  -  ROS Melodic desktop
+ Ubuntu 20.04  -  ROS Noetic desktop

#### 2.2 ROS依赖 

```bash
sudo apt-get install ros-$ROS_DISTRO-pcl-ros ros-$ROS_DISTRO-pcl-conversions  ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-message-generation
```

#### 3.2 其他依赖

~~~bash
sudo apt-get install libpcap-dev libyaml-cpp-dev libeigen3-dev libboost${BOOST_VERSION}-dev
~~~



## 3.编译运行

#### 3.1 编译

~~~bash
#创建工作空间及src目录  lslidar_ws为工作空间名 可自定义
mkdir -p ~/lidar_ws/src
#将驱动压缩包解压缩放到~/lslidar_ws/src 目录下
#返回工作空间
cd ~/lslidar_ws
#编译及刷新终端环境
catkin_make
source devel/setup.bash
~~~

#### 3.2 运行

运行单个雷达:

~~~bash
roslaunch lslidar_driver lslidar_x10.launch # 单线雷达

roslaunch lslidar_driver lslidar_cx.launch  # 机械式雷达

roslaunch lslidar_driver lslidar_ch.launch  # 905雷达

roslaunch lslidar_driver lslidar_ls.launch  # 1550雷达
~~~

运行多个雷达：

~~~bash
# 可根据实际情况自定义launch文件
roslaunch lslidar_driver lslidar_double.launch
~~~





## 4.参数介绍

**lslidar_cx.launch**文件内容如下，更多文件在`lslidar_driver/launch`文件夹下。

每个参数含义见注释说明或咨询**`技术支持`**。

~~~xml
<launch>
  <arg name="device_ip" default="192.168.1.200"/>   <!-- 雷达ip -->
  <arg name="msop_port" default="2368"/>            <!-- 雷达目的数据端口 -->
  <arg name="difop_port" default="2369"/>           <!-- 雷达目的设备端口 -->
  <arg name="pcl_type" default="false"/>            <!-- 点云类型   true: xyzi -->
  <arg name="use_time_service" default="false"/>    <!-- 雷达是否使用授时(GPS PTP NTP) -->
  <arg name="use_first_point_time" default="false"/><!-- true: 使用每帧第一个点的时间做为点云时间-->
  <arg name="use_absolute_time" default="false"/>   <!-- 点时间格式 true: 使用绝对时间  注意与点云头文件中USE_ABSOLUTE_TIME保持一致-->
  <arg name="packet_rate" default="1695.0"/>     <!-- PCAP文件回放速率，离线解析pcap包时使用 -->

  <node pkg="lslidar_driver" type="lslidar_driver_node" name="lslidar_driver_node" output="screen" ns="cx">
    <!-- 离线解析PCAP数据时打开注释 --> 
    <!-- <param name="pcap" value="$(find lslidar_driver)/pcap/xxx.pcap"/> -->  
    <!-- 自定义线束角度裁剪文件 -->
    <param name="filter_angle_file" value="$(find lslidar_driver)/param/filter_angle.yaml"/> 
    <param name="use_time_service" value="$(arg use_time_service)"/>
    <param name="packet_rate" value="$(arg packet_rate)"/>
    <param name="lidar_type" value="CX"/>			<!-- 雷达类型 -->
    <param name="device_ip" value="$(arg device_ip)"/>
    <param name="msop_port" value="$(arg msop_port)"/>
    <param name="difop_port" value="$(arg difop_port)"/>
    <param name="pcl_type" value="$(arg pcl_type)"/>
    <param name="add_multicast" value="false"/>     <!-- 雷达是否开启组播 true:使用组播模式-->
    <param name="group_ip" value="224.1.1.2"/>      <!-- 雷达组播ip -->
    <param name="frame_id" value="laser_link"/>     <!-- 点云帧id -->
    <param name="pointcloud_topic" value="lslidar_point_cloud"/>  <!-- 发布点云话题名 -->
    <param name="use_first_point_time" value="$(arg use_first_point_time)"/> 
    <param name="use_absolute_time" value="$(arg use_absolute_time)"/> 
    <param name="min_range" value="0.15"/>      <!-- 雷达扫描最小距离 小于该值的点将被过滤 -->
    <param name="max_range" value="200.0"/>     <!-- 雷达扫描最大距离 大于该值的点将被过滤 -->
    <param name="angle_disable_min" value="0"/> <!-- 雷达扫描最小裁剪角度 填整数 单位: 0.01° -->
    <param name="angle_disable_max" value="0"/> <!-- 雷达扫描最大裁剪角度 填整数 单位: 0.01° -->
    <param name="horizontal_angle_resolution" value="0.18"/>  <!--10Hz:0.18  20Hz:0.36 5Hz:0.09 -->
    <param name="publish_scan" value="false"/>      <!-- 是否发布 LaserScan 数据 -->
    <param name="scan_num" value="8"/>              <!-- LaserScan线号 -->
    
    <param name="is_pretreatment" value="false"/> <!--是否开启预处理 仅对点云生效-->
    <param name="x_offset" value="0.0"/>  <!-- rviz 中表现为围绕红色的 x 轴偏移量 单位: m-->
    <param name="y_offset" value="0.0"/>  <!-- rviz 中表现为围绕绿色的 y 轴偏移量 单位: m-->
    <param name="z_offset" value="0.0"/>  <!-- rviz 中表现为围绕蓝色的 z 轴偏移量 单位: m-->
    <param name="roll" value="0.0"/>      <!-- rviz 中表现为围绕红色的 x 轴旋转   单位: rad-->
    <param name="pitch" value="0.0"/>     <!-- rviz 中表现为围绕绿色的 y 轴旋转   单位: rad-->
    <param name="yaw" value="0.0"/>       <!-- rviz 中表现为围绕蓝色的 z 轴旋转   单位: rad-->
  </node>

  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find lslidar_driver)/rviz/lslidar.rviz"/> <!-- 注释此行不启动RViz -->
 
  <!-- <node pkg="tf" type="static_transform_publisher" name="laser_link_to_world" args="0 0 0 0 0 0 world laser_link 100"/> -->	<!-- tf静态坐标转换 -->
</launch>
~~~

### 主要参数说明：

- **lidar_type**

  雷达类型，单线激光雷达为**`X10`**，机械式激光雷达为**`CX`**，905激光雷达为**`CH`**，1550激光雷达为**`LS`**。

- **lidar_model**

  雷达型号，激光雷达具体型号，如：M10, CX126S3, LSS3, LSS4等。

- **device_ip**

  雷达设备IP地址(注意不是雷达目的IP地址)，可使用`wireshark`或`tcpdump`查看。

- **msop_port**

  雷达目的数据端口(源端口: 2369)，可使用`wireshark`或`tcpdump`查看。

- **difop_port**

  雷达目的设备端口(源端口: 2368)，可使用`wireshark`或`tcpdump`查看。

- **use_time_service**

  授时功能(请确保授时源稳定，否则影响点云时间连续性)。

  - true:  使用雷达时间(GPS, NTP, PTP)
  - false: 使用系统时间

- **use_first_point_time**

  点云时间

  - true:  使用每帧第一个点的时间做为点云时间
  - false: 使用每帧最后一个点的时间做为点云时间

- **use_absolute_time**

  点的时间模式(注意此值要与**`lslidar_pointcloud.hpp`**文件中**`USE_ABSOLUTE_TIME`**保持一致)

  - true:  点的时间使用绝对时间格式
  - false: 点的时间使用相对时间格式

- **packet_rate**

  PCAP文件回放速率，每秒读取数据包个数。雷达每秒包数可使用`wireshark`统计下的`I/O图表`查看。

- **frame_id**

  程序发布点云数据坐标系名称，可通过`tf`实现坐标系间的变换。

- **pointcloud_topic**

  程序发布点云数据话题名称。

- **min_range  max_range**

  - **`min_range`**： 点云起始距离

  - **`max_range`**：点云结束距离

    *此设置为软件屏蔽，会将区域外的点过滤。 单位：1m*

- **scan_start_angle  scan_end_angle  angle_disable_min  angle_disable_max**

  - **`scan_start_angle`**: 点云起始水平角度

  - **`scan_end_angle`**:  点云结束水平角度

  - **`angle_disable_min`**: 点云起始水平角度裁剪

  - **`angle_disable_max`**: 点云结束水平角度裁剪

    *此设置为软件屏蔽，会将区域外的点过滤。 单位：0.01° 填整数*

- **is_pretreatment**

  点云预处理，此值为ture时点云数据根据以下参数进行空间位置和方向的变换。

  - **`x_offset`**: x 轴偏移量 单位: m
  - **`y_offset`**: y 轴偏移量 单位: m
  - **`z_offset`**: z 轴偏移量 单位: m
  - **`roll`**:    x 轴旋转   单位: rad
  - **`pitch`**:  y 轴旋转   单位: rad
  - **`yaw`**:      z 轴旋转   单位: rad

  Please grant the corresponding serial port permissions.



### 特殊参数说明：

#### 以下功能仅对特定系列雷达支持

- **serial_port**

  激光雷达连接的串口名称（例如 `/dev/ttyUSB0` 或 `/dev/ttyACM0`），**使用串口雷达时，请填写正确的串口名称，并确保已授予该串口的读写权限**。

- **use_high_precision**

  高精度模式，laserscan数据启用高精度模式，角度分辨率减小10倍，数据量增加10倍，提高精度。

- **publish_multiecholaserscan**

  N10Plus雷达是否发布`sensor_msgs::MultiEchoLaserScan`数据，此值为true时发布数据。

- **enable_noise_filter**

  孤立噪点滤波开关，此值为true时将过滤孤立点(N10Plus不生效)。

- **pcl_type**

  点云数据类型，此值为true时使用 x y z i 类型发布点云数据(坐标，强度)。

- **filter_angle_file**

  - **`disable_min`**: 小于此角度的点云数据将被过滤掉
  - **`disable_max`**: 小于此角度的点云数据将被过滤掉

  单独线号自定义角度裁剪，允许对每条扫描线分别设置裁剪角度范围。可根据实际使用情况更改**`lslidar_driver/param/filter_angle.yaml`**文件。

- **publish_scan**

  发布 `LaserScan` 数据，此值为true时发布数据。

- **scan_num**

  指定用于发布`LaserScan`数据的线号，线号范围根据雷达扫描线束而定。

- **echo_mode**

  回波模式，0:发布全部点云  1:发布第一次回波点云  2:发布第二次回波点云(双回波模式下生效)。

- **is_add_frame**

  叠帧发布点云信息，将连续两帧点云数据叠加后一起发布。

- **packet_loss**

  丢包检测，开启后驱动将雷达丢包总数以话题的形式发布，话题名字为**`packet_loss`**，消息类型为**`std_msgs::Int64`**。

  

​		

### 组播模式：

- 将雷达目的IP设置为组播网段

- 修改launch文件以下参数

  ~~~shell
  <param name="add_multicast" value="false"/>               #是否添加组播
  <param name="group_ip" value="224.1.1.2"/>                #组播的ip
  ~~~

- 运行以下指令将电脑加入组内（将指令中的enp2s0替换为用户电脑的网卡名,可用ifconfig查看网卡名)

  ~~~shell
  ifconfig
  sudo route add -net 224.0.0.0/4 dev enp2s0
  ~~~



### 离线播放PCAP模式：

- 把录制好的`PCAP`文件，拷贝到**`lslidar_driver/pcap`**文件夹下。

- 修改launch文件以下参数

  ~~~shell
  #pcap包路径，加载pcap包时打开此注释
  <param name="pcap" value="$(find lslidar_cx_driver)/pcap/xxx.pcap" />   
  ~~~



###  pcl点云类型：

- 驱动发布点云为自定义点云类型，定义参考**`lslidar_driver/include/lslidar_pointcloud.hpp`**

- 如果使用绝对时间，请取消 **`#define USE_ABSOLUTE_TIME`**的注释

  ~~~c++
  struct PointXYZIRT {
      PCL_ADD_POINT4D;      // x, y, z 和 data[4]
      PCL_ADD_INTENSITY;    // 强度
      std::uint16_t ring;   // 线号
      POINT_TIME_TYPE time; // 时间  POINT_TIME_TYPE -> float/double 
  
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
  ~~~



### 设备信息：

- 雷达驱动将设备信息以话题的形式发布，话题名字为`lslidar_device_info`，消息类型为`lslidar_msgs::LslidarInformation`定义参考**`lslidar_msgs/LslidarInformation.msg`**



### 故障码：

- LS(1550)系列雷达驱动将故障码以话题的形式发布，话题名字为`lslidar_fault_code`，消息类型为`std_msgs::String`

  *每位故障码含义请联系技术支持*



### 点云时间：

- 雷达驱动将点云时间以话题的形式发布，话题名字为`time_topic`，消息类型为`std_msgs::Float64`





## 5.雷达功能设置与调控

#### **部分功能只对特定系列雷达支持。**

### 设置雷达网络配置：

~~~bash
# 新开一个终端 cx为命名空间
source devel/setup.bash

# 使用显示传参配置雷达
rosservice call /cx/network_setup "lidar_ip: '192.168.1.200'
destination_ip: '192.168.1.102'
data_port: 2368
dev_port: 2369" 

# 使用配置文件参数来配置雷达(network_setup.yaml)
rosservice call /cx/network_setup "lidar_ip: ''
destination_ip: ''
data_port: 0
dev_port: 0" 
~~~

#### ***更改后需重启驱动***





### 雷达授时方式：

~~~bash
# 新开一个终端	cx为命名空间
source devel/setup.bash

# 0: GPS   1: PTP L2   2: NTP   3: PTP UDPv4   4: E2E L2   5: E2E UDPv4
rosservice call /cx/time_mode "time_mode: 0
ntp_ip: ''" 

# 设置NTP授时是需要填写ntp ip
rosservice call /cx/time_mode "time_mode: 2
ntp_ip: '192.168.1.102'"
~~~





### 雷达上下电(机械式)：

~~~bash
# 新开一个终端 cx为命名空间
source devel/setup.bash
~~~

上电：

~~~bash
rosservice call /cx/power_control "power_control: 1"
~~~

下电：

~~~bash
rosservice call /cx/power_control "power_control: 0"
~~~





### 雷达转动/停止转动(机械式)：

~~~bash
# 新开一个终端 cx为命名空间
source devel/setup.bash
~~~

转动：

~~~bash
rosservice call /cx/motor_control "motor_control: 1"
~~~

停止转动：

~~~bash
rosservice call /cx/motor_control "motor_control: 0"
~~~





### 雷达转速(机械式 905)：

~~~bash
# 新开一个终端 cx为命名空间
source devel/setup.bash
# 可选频率  5Hz/10Hz/20Hz
rosservice call /cx/motor_speed "motor_speed: 20"	# 5 10 20  (扩展30 40 50 60仅只有CX1S3可设置)
~~~





### 雷达去除雨雾尘等级(机械式)：

~~~bash
# 新开一个终端 cx为命名空间
source devel/setup.bash
# 可选等级  0/1/2/3 ，0-3 数字越大，去除越强
rosservice call /cx/remove_rain_fog_dust "rfd_removal: 3"
~~~





### 雷达去拖尾等级(机械式)：

~~~bash
# 新开一个终端 cx为命名空间
source devel/setup.bash
# 可选等级 0-10 数字越大，去除越强
rosservice call /cx/tail_remove "tail_removal: 4"	#旧版本雷达最大为4
~~~





### 雷达角度畸变矫正(LSS3)：

**LSS4系列雷达暂无此功能，调用此服务将配置是否发送无效数据**

~~~bash
# 新开一个终端 ls为命名空间
source devel/setup.bash
~~~

关闭角度畸变矫正：

~~~bash
rosservice call /ls/angle_distortion_correction "angle_distortion_correction: 0" 
~~~

开启角度畸变矫正：

~~~bash
rosservice call /ls/angle_distortion_correction "angle_distortion_correction: 1" 
~~~

**开启此功能可减少CPU占用**

#### *更改后需重启驱动*





### 雷达帧率(1550)：

~~~bash
# 新开一个终端 ls为命名空间
source devel/setup.bash
# 0: 正常帧率    1: 50%帧率    2: 25%帧率
rosservice call /ls/frame_rate "frame_rate: 1"		# 50%帧率  5hz
~~~







### **雷达无效数据发送**(1550)：

**LSS4系列雷达此功能请调用`angle_distortion_correction`服务**

~~~bash
# 新开一个终端 ls为命名空间
source devel/setup.bash
~~~

发送无效数据：

~~~bash
rosservice call /ls/invalid_data "invalid_data: 0"
~~~

不发送无效数据：

~~~bash
rosservice call /ls/invalid_data "invalid_data: 1"
~~~

***不发送无效数据可以减少CPU占用，但会导致点云时间不连续***





### 雷达待机模式(1550)：

~~~bash
# 新开一个终端 ls为命名空间
source devel/setup.bash
~~~

正常模式：

~~~bash
rosservice call /ls/standby_mode "standby_mode: 0"
~~~

待机模式：

~~~bash
rosservice call /ls/standby_mode "standby_mode: 1"
~~~





### 单线雷达电机控制：

~~~bash
# N301暂不支持
# 新开一个终端 x10为命名空间
source devel/setup.bash
~~~

转动：

~~~bash
rostopic pub -1 /x10/motor_control std_msgs/Int8 "data: 1"
~~~

停转并不发数据：

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

1. 新增兼容CH32A雷达
2. 新增点云时间话题发布
3. 更新C16国产化垂直角度

Date    : 2025-02-24

--------------------------------------------------------------------



update version : LSLIDAR_ROS_V5.1.0_250324

Modify: 

1. 新增兼容M10, M10GPS, M10P, N10, N301雷达
1. 新增单线雷达电机控制功能
1. 新增兼容1550 LSS4系列雷达

Date    : 2025-03-24

--------------------------------------------------------------------



update version : LSLIDAR_ROS_V5.1.1_250403

Modify: 

1. 新增兼容N10Plus雷达
1. N10Plus雷达新增MultiEchoLaserScan消息
1. 新增laserscan高精度模式
1. 新增点云时间与点时间格式选择

Date    : 2025-04-03

--------------------------------------------------------------------



update version : LSLIDAR_ROS_V5.1.2_250427

Modify: 

1. 新增兼容 MS32L 雷达
1. 更新机械式C32 5.0雷达垂直角度

Date    : 2025-04-27

--------------------------------------------------------------------

update version : LSLIDAR_ROS_V5.1.2_250507

Modify: 

1. 新增git 的管理

Date    : 2025-05-07

------

update version : LSLIDAR_ROS_V5.1.3_250704

Modify: 

1. 修改机械式v4.0 PTP授时 时间不正确的问题

Date    : 2025-07-04

------

update version : LSLIDAR_ROS_V5.1.4_250724

Modify: 

1. 增加CH1W雷达型号的兼容

Date    : 2025-07-24

------

update version : LSLIDAR_ROS_V5.1.5_250822

Modify: 

1. 扩展转速设置 (扩展到30 40 50 60仅只有CX1S3可设置)

Date    : 2025-08-22

------

