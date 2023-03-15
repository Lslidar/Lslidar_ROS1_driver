<img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/%E7%94%BB%E6%9D%BF%201%404x-100(1).jpg" width="1200px" />

# Instructions

[LEISHEN-Lidar Offical](http://www.lslidar.com/)

## Preparation

### Supported Operating System

Unbuntu 16.04 + ROS Kinetic

Ubuntu 18.04 + ROS Melodic

### ROS Installation

[Ubuntu](http://wiki.ros.org/Installation/Ubuntu)

_*Before starting this tutorial, please complete the installation. This tutotial assumes that Ubuntu is being used._

### Create a ROS workspace

[Create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 

### Install Git and initialize it

1. [Install Git](https://github.com/git-guides/install-git)
2. Go into the destinated file via terminal, type **git init** to initialize git.

```
cd <your_work_space>
git init
```

## Clone Lidar's related documents/repository

_**Operation Instructions:** To access the operation intructions, please click on the blue "link" next to the title of Lidar and read through the "README.md" file in the repository._

_**Clone:** To clone each Lidar's repository, please copy and paste the corresponding code in the terminal_



### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/LS180S1.png" width="50px" />LS128/180/400S1_V1.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/LS128/180/400S1_V1.0)

Linux/Mac: `git clone -b LS128/180/400S1_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch LS128/180/400S1_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`



### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/LS128S2.png" width="50px" />LS128/180/400S2_V1.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/LS128/180/400S2_V1.0)

Linux/Mac: `git clone -b LS128/180/400S2_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch LS128/180/400S2_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/C16-V3.0.png" width="50px" />C16_V2.6/2.8/3.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/C16_V2.6/2.8/3.0)

Linux/Mac: `git clone -b C16_V2.6/2.8/3.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch C16_V2.6/2.8/3.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/C16-V3.0.png" width="50px" />C32_V2.6/2.8/3.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/C32_V2.6/2.8/3.0)

Linux/Mac: `git clone -b C32_V2.6/2.8/3.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch C32_V2.6/2.8/3.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`


### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/C16.png" width="50px" />C1_v4.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/C1_v4.0)

Linux/Mac: `git clone -b C1_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch C1_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/C16.png" width="50px" />C8_v4.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/C8_v4.0)

Linux/Mac: `git clone -b C8_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch C8_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/C16.png" width="50px" />C16_v4.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/C32_v2.6)

Linux/Mac: `git clone -b C16_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch C16_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/C16.png" width="50px" />C32_v4.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/C32_v3.0)

Linux/Mac: `git clone -b C32_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch C32_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`


### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/C32W.png" width="50px" />C32W_v4.0[link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/C32W_v4.0)

Linux/Mac: `git clone -b C32W_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch C32W_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/CH32R.png" width="50px" />CH32R_V4.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/CH32R_V4.0)

Linux/Mac: `git clone -b CH32R_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch CH32R_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`


### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/CH64W.png" width="50px" />CH1W_V1.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/CH1W_V1.0)

Linux/Mac: `git clone -b CH1W_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch CH1W_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/CH128X1.png" width="50px" />CH16X1_V1.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/CH16X1_V1.0)

Linux/Mac: `git clone -b CH16X1_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch CH16X1_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/CH128X1.png" width="50px" />CH128X1_V1.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/CH128X1_V1.0)

Linux/Mac: `git clone -b CH128X1_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch CH128X1_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`


### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/CH128X1.png" width="50px"/>CH128S1_V1.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/CH128S1_V1.0)

Linux/Mac: `git clone -b CH128S1_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch CH128S1_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/CH64W.png" width="50px"/>CH64W_V1.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/CH64W_V1.0)

Linux/Mac: `git clone -b CH64W_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch CH64W_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/CH64W.png" width="50px" />CB64S1_V1.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/CB64S1_V1.0)

Linux/Mac: `git clone -b CB64S1_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch CB64S1_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/CX128S2.png" width="50px" />CX128S2 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/CX128S2)

Linux/Mac: `git clone -b CX128S2 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch CX128S2 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/HS1.png" width="50px" />HS1 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/HS1)

Linux/Mac: `git clone -b HS1 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch HS1 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/M10P.png" width="50px" />M10_V1.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/M10_V1.0)

Linux/Mac: `git clone -b M10_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch M10_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/N10.png" width="50px" />N10_V1.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/N10_V1.0)

Linux/Mac: `git clone -b N0_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch N10_V1.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/N401-P.png" width="50px" />N301-P_V5.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/N301-P_V5.0)

Linux/Mac: `git clone -b N301-P_V5.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch N301-P_V5.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/N401-P.png" width="50px" />N401-P_V5.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/N401-P_V5.0)

Linux/Mac: `git clone -b N401-P_V5.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch N401-P_V5.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/C16.png" width="50px" />MS-C16_V4.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/MS-C16_V4.0)

Linux/Mac: `git clone -b MS-C16_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch MS-C16_V4.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

### <img src="https://github.com/Lslidar/Lslidar_ROS1_driver/blob/master/image/W Series.png" width="50px" />W_Series_V5.0 [link](https://github.com/Lslidar/Lslidar_ROS1_driver/tree/W_Series_V5.0)

Linux/Mac: `git clone -b W_Series_V5.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

Windows: `git clone --branch W_Series_V5.0 https://github.com/Lslidar/Lslidar_ROS1_driver.git`

FAQ
----

Techinical Support
----

Any more question please commit an issue

you can contact support@lslidar.com
or Enter our live chat window
[Customer service entrance](https://1893520.s5.udesk.cn/im_client/?web_plugin_id=502&language=en-us)
