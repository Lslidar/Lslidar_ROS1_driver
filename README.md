ROS Installation
-----

[ubuntu](http://wiki.ros.org/Installation/Ubuntu)

Before starting this turorial, please complete installation . This tutorial assumes that Ubuntu is being used.

# lslidar_Ls01b_v1.0
[Customer service entrance](https://1893520.s5.udesk.cn/im_client/?web_plugin_id=502)
## Description

The `lslidar_Ls01b_v1.0` package is a linux ROS driver for lslidar Ls01b_v1.0.

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
git clone -b ls01b_v1.0 https://github.com/Lslidar/lslidar_ros.git
catkin_make
source devel/setup.bash
```

## View Data

1. Launch the provided pointcloud generation launch file.

```
roslaunch ls01b_v2 ls01b_v2.launch
```

1. Launch rviz, with the "laser_link" frame as the fixed frame.

```
rosrun rviz rviz -f laser_link
```

1. In the "displays" panel, click `Add`, click`By topic`,then select `LaserScan`, then press `OK`.

2. In the "Topic" field of the new `LaserScan` tab, enter `/scan`.

### **Parameters**

`scan_topic` (`string`, `default: scan`)

Lidar's data topic name. Default value: scan

`frame_id` (`string`, `default: laser_link`)

Default value: laser_link. Point cloud coordinates name.

`serial_port` (`string`, `default: /dev/ttyUSB0`)

Serial port. Default value: /dev/ttyUSB0

`baud_rate` (`int`, `default: 460800`)

Serial port baud rate. Default value: 460800

`angle_resolution` (`double`, `default: 0.25`)

Lidar's angle resolution. 

`zero_as_max` (`bool`, `default: false`)

Whether to enable the function of setting the undetectable area to the maximum value. Default value: false (true: yes; false: no). 

`min_as_zero` (`bool`, `default: false`)

Default value: false (false: the undetectable area is inf; true: the undetectable area is 0)

`rpm` (`int`, `default: 600`)

Lidar rotate speed. Default value: 600R/M.

`angle_compensate` (`bool`, `default: true`)

Whether to enable the non-coaxial angle compensation. Default value: true (true: yes; false: no). 

**Published Topics**

`parameter descriptions`

this is publish the truncated lidar data value

`scan` (`lslidar_LS01B_V2_msgs/LaserScan`)

This is published the LaserScan topic.

**Node**

```
roslaunch ls01b_v2 ls01b_v2.launch
```

Note that this launch file launches both the driver and the decoder, which is the only launch file needed to be used.


## FAQ

## Technical support

Any more question please commit an issue.

you can contact support@lslidar.com
or Enter our live chat window
[Customer service entrance](https://1893520.s5.udesk.cn/im_client/?web_plugin_id=502)



****
