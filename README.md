ROS Installation
-----

[ubuntu](http://wiki.ros.org/Installation/Ubuntu)

Before starting this turorial, please complete installation . This tutorial assumes that Ubuntu is being used.

# lslidar_Ls01d_v1.0
[Customer service entrance](https://1893520.s5.udesk.cn/im_client/?web_plugin_id=502)
## Description

The `lslidar_Ls01d_v1.0` package is a linux ROS driver for lslidar Ls01d_v1.0.

Supported Operating
----

Ubuntu 16.04 Kinetic
Ubuntu 18.04 Melodic

## Connect to the lidar

1. Connect the lidar to the included USB adapter board
2. Use a USB cable to connect the USB adapter board to the computer USB port
3. Use `sudo ls -l /dev/ttyUSB*` command to find serial port name <br>Use `sudo chmod 777 /dev/serialportname` to give permissions to the serial port<br>

## Compiling

This is a Catkin package. Make sure the package is on `ROS_PACKAGE_PATH`  after cloning the package to your workspace. And the normal procedure for compiling a catkin package will work.

```
cd your_work_space
cd src
git clone –b Ls01d_v1.0 https://github.com/Lslidar/lslidar_ros
catkin_make
source devel/setup.bash
```

## View Data

1. Launch the provided pointcloud generation launch file.

```
roslaunch ls01d ls01d.launch
```

1. Launch rviz, with the "laser_link" frame as the fixed frame.

```
rosrun rviz rviz -f laser_link
```

1. In the "displays" panel, click `Add`, click`By topic`,then select `LaserScan`, then press `OK`.

2. In the "Topic" field of the new `LaserScan` tab, enter `/scan`.

### **Parameters**

`serial_port` (`string`, `default: /dev/ttyUSB0`)

Serial port. Default value: /dev/ttyUSB0

`scan_topic` (`string`, `default: scan`)

Lidar's data topic name. Default value: scan

`laser_link` (`string`, `default: laser_link`)

Lidar coordinates name. Default value: laser_link.

**Published Topics**

`parameter descriptions`

this is publish the truncated lidar data value

`scan` (`lslidar_Ls01d_V2_msgs/LaserScan`)

This is published the LaserScan topic.

**Node**

```
roslaunch ls01d_v2 ls01d_v2.launch
```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ

## Technical support

Any more question please commit an issue.

you can contact support@lslidar.com
or Enter our live chat window
[Customer service entrance](https://1893520.s5.udesk.cn/im_client/?web_plugin_id=502)
