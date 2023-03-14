/******************************************************************************
 * This file is part of lslidar driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef LSLIDAR_Ch_DRIVER_H
#define LSLIDAR_Ch_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string.h>
//#include <boost/shared_ptr.hpp>
//#include <boost/thread.hpp>
#include <thread>
#include <memory>
#include <ros/ros.h>
#include "input.h"

namespace lslidar_ch_driver {

class LslidarChDriver {
public:

    LslidarChDriver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarChDriver();
    bool initialize();
    bool polling();
    void difopPoll(void);
    void initTimeStamp(void);
    //void getFPGA_GPSTimeStamp(lslidar_ch1w_msgs::LslidarChPacketPtr &packet);
    typedef std::shared_ptr<LslidarChDriver> LslidarChDriverPtr;
    typedef std::shared_ptr<const LslidarChDriver> LslidarChDriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();

    //socket Parameters
    int msop_udp_port;
    int difop_udp_port;

    std::shared_ptr<Input> msop_input_;
    std::shared_ptr<Input> difop_input_;

    // Converter convtor_
    std::shared_ptr<std::thread> difop_thread_;

    // Ethernet relate variables
    std::string lidar_ip_string;
    std::string group_ip_string;
    std::string frame_id;
	std::string dump_file;

    in_addr lidar_ip;
    int UDP_PORT_NUMBER;
    int socket_id;
    bool add_multicast;
    bool prism_flag;
    float prism_angle[4];
    float prism_offset;

    // ROS related variables
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Publisher packet_pub;



    // add for time synchronization
    bool use_time_service;
    uint64_t pointcloudTimeStamp;

    unsigned char packetTimeStamp[10];
    struct tm cur_time;
    unsigned short int us;
    unsigned short int ms;
    ros::Time timeStamp;
};

typedef LslidarChDriver::LslidarChDriverPtr LslidarChDriverPtr;
typedef LslidarChDriver::LslidarChDriverConstPtr LslidarChDriverConstPtr;

} // namespace lslidar_driver

#endif // _LSLIDAR_Ch_DRIVER_H_
