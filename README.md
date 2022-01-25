ROS Installation
-----

[ubuntu](http://wiki.ros.org/Installation/Ubuntu)

Before starting this turorial, please complete installation . This tutorial assumes that Ubuntu is being used.

# lslidar_HS4_V1.0

## Description

The `lslidar_HS4_v1.0` package is a linux ROS driver for lslidar HS4_ V1.0.

Supported Operating
----

Ubuntu 16.04 Kinetic
Ubuntu 18.04 Melodic

## Connect to the lidar

1. Power the lidar via the included adapter
2. Connect the lidar to an Ethernet port on your computer.
3. Assign the computer IP based on the default DEST IP `192.168.1.102.` <br>`sudo ifconfig eth0 192.168.1.102`（eth0 is the network card name ）<br>

## Compiling

This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH`  after cloning the package to your workspace. And the normal procedure for compiling a catkin package will work.

```
cd your_work_space<br>
cd src<br>
git clone –b HS4_v1.0 https://github.com/lsLIDAR/lslidar_ros/HS4_v1.0<br>
catkin_make<br>
source devel/setup.bash/<br>
```

## View Data

1. Launch the provided pointcloud generation launch file.

```
roslaunch lslidar_ch_decoder lslidar_ch.launch
```

1. Launch rviz, with the "laser_link" frame as the fixed frame.

```
rosrun rviz rviz -f laser_link
```

1. In the "displays" panel, click `Add`, click`By topic`,then select `pointcloud2`, then press `OK`.

2. In the "Topic" field of the new `pointcloud2` tab, enter `/lslidar_point_cloud`.

### **Parameters**

`device_ip` (`string`, `default: 192.168.1.200`)

By default, the IP address of the device is 192.168.1.200.

`msop_port`(`int`, `default:2368`)

Default value: 2368. Data package port. Modifiable, please keep it consistent with the data package port set by the host computer. 

`difop_port`(`int`, `default:2369`)

Default value:2369. Device package port. Modifiable, please keep it consistent with the device package port set by the host computer. 

`time_synchronization` (`bool`, `default: true`)

Whether to open the external time synchronization (pre-configuration required). Default value: true (true: yes; false: no). 


### lslidar_ch_driver

`add_multicast`(`bool`,`default: false`)

Default value: false (true: yes; false: no). Whether to switch to the multicast mode. 

`group_ip`(`string`,`default:224.1.1.2`)

Default value: 224.1.1.2. Multicast IP. Enabled when the value of add_multicast is "true".


### lslidar_ch_decoder

`frame_id` (`string`, `default: laser_link`)

Default value: laser_link. Point cloud coordinates name.

`point_num` (`int`, `default: 2000`)

Point cloud number in each frame. Different under various frequency. Default value: 2000.

`min_range` (`double`, `default: 0.05`)

The minimum scanning range. Point cloud data inside this range would be removed. Default value: 0.05 meters.

`max_range` (`double`, `default: 300.0`)

The maximum scanning range. Point cloud data outside this range would be removed. Default value: 300 meters.

`lslidar_point_cloud` (`string`, `default: lslidar_point_cloud`)

Topic name of pointcloud2, modifiable.

`frequency` (`double`, `default: 10.0`)

Lidar scanning frequency. Default value: 10.0Hz.

`return_model` (`int`, `default: 0`)

Lidar's return mode. Default value: 0 (0 represents single return mode)

`publish_point_cloud` (`bool`, `default: true`)

Publish pointcloud2 topic. Default value: true (true: yes; false: no)

`use_gps_ts` (`bool`, `default: false`)

Whether to use GPS time synchronization (pre-configuration required). Default value: false (true: yes; false: no).

**Published Topics**

`lslidar_point_cloud` (`sensor_msgs/PointCloud2`)

This is only published when the `publish_point_cloud` is set to `true` in the launch file.

`lslidar_packets` (`lslidar_ch_msgs/LslidarchPacket`)

Each message corresponds to a lslidar packet sent by the device through the Ethernet.

**Node**

```
roslaunch lslidar_ch_decoder lslidar_ch.launch
```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ

## Technical support

Any more question please commit an issue.

Or connect support@lslidar.com

****