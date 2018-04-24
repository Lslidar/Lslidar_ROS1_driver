#!/usr/bin/python
# -*- coding: utf-8 -*-
# @Time    : 18-4-23 下午5:25
# @Author  : Yutong
# @Email   : 416178264@qq.com
# @Site    : 
# @File    : Test_MultiChannel.py
# @Software: PyCharm
# @Desc    :

import rospy
from lslidar_c16_msgs.msg import LslidarC16Layer
import os
import numpy as np
import matplotlib.pyplot as plt

Remote_IP = '192.168.0.138'

ROS_IP='http://'+Remote_IP+':11311'
os.environ['ROS_MASTER_URI']=ROS_IP  #environ的键值必须是字符串
os.system('echo $ROS_MASTER_URI')


if __name__ == '__main__':
    rospy.init_node("multi_channel")
    msg = rospy.wait_for_message("scan_channel", LslidarC16Layer)
    data_len = len(msg.scan_channel)

    for i in range(data_len):
        laser_msg = msg.scan_channel[i]
        laser_distance = np.array(laser_msg.ranges)
        counter = len(laser_distance)
        laser_distance[laser_distance == np.inf] = 0.0
        laser_angle = np.linspace(laser_msg.angle_min, laser_msg.angle_max, counter)
        intensity = np.array(laser_msg.intensities)
        x_laser = np.multiply(laser_distance, np.cos(laser_angle))
        y_laser = np.multiply(laser_distance, np.sin(laser_angle))
        plt.plot(x_laser, y_laser, 'b.')
        plt.show()