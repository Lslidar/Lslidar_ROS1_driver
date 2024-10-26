/******************************************************************************
 * This file is part of lslidar_cx driver.
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
#ifndef _LS_N301_DRIVER_H_
#define _LS_N301_DRIVER_H_

#define DEG_TO_RAD 0.017453f
#define RAD_TO_DEG 57.29577f

class Request;

#include "input.h"
#include "ThreadPool.h"

#include <string>
#include <ros/ros.h>
#include <functional>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/LaserScan.h>
#include <regex>
#include <atomic>
#include <memory>
#include <mutex>
#include <algorithm>

namespace lslidar_driver {
//raw lslidar packet constants and structures
    static const int BLOCKS_PER_PACKET = 12;            // 每个数据包中数据块个数
    static const int SIZE_BLOCK = 100;                  // 每个数据块长度   2字节标准位 2字节方位角 6字节时间 90字节点信息
    static const int RAW_SCAN_SIZE = 3;                 // 每个点长度 距离2字节  强度1字节
    static const int SCANS_PER_PACKET_MAX = 360;        // 每包最大有效点数360个

// special defines for lslidarlidar support
    static const float DISTANCE_RESOLUTION_1_7 = 0.004f;   // 1.7协议 距离分辨率
    static const int SCANS_PER_FIRING_1_7 = 31;
    static const int SCANS_PER_PACKET_1_7 = 360;           // 1.7协议 每包最大有效点数360个
    static const int FIRING_TOFFSET_1_7 = 30;              // 1.7协议 每个数据块中点数
    
    static const float DISTANCE_RESOLUTION_1_6 = 0.002f;   // 1.6协议 距离分辨率
    static const int SCANS_PER_PACKET_1_6 = 24;            // 1.6协议 每包最大有效点数360个

    struct FiringCX {
        uint16_t firing_azimuth[BLOCKS_PER_PACKET];
        int azimuth[SCANS_PER_PACKET_MAX];
        float distance[SCANS_PER_PACKET_MAX];
        float intensity[SCANS_PER_PACKET_MAX];
    };

    class LslidarDriver {
    public:
        LslidarDriver(ros::NodeHandle &node, ros::NodeHandle &private_nh);

        ~LslidarDriver() {}

        bool checkPacketValidity(lslidar_n301_driver::LslidarPacketPtr &packet);
        
        bool loadParameters();

        void initTimeStamp();

        bool createRosIO();

        void difopPoll();

        void publishScan();

        bool poll();

        bool initialize();

        bool determineLidarType();

    private:

        std::function<void(lslidar_n301_driver::LslidarPacketPtr &)> decodePacket;

        void decodePacket_1_6(lslidar_n301_driver::LslidarPacketPtr &packet);
        
        void decodePacket_1_7(lslidar_n301_driver::LslidarPacketPtr &packet);

    public:
        FiringCX firings;

        int protocol;
        int msop_udp_port{};
        int difop_udp_port{};
        int point_num{};
        int angle_disable_min{};
        int angle_disable_max{};
        int angle_able_min{};
        int angle_able_max{};
        int SCANS_PER_PACKET;
        uint16_t year;
        uint16_t month;
        uint16_t day;
        uint16_t last_azimuth;
        uint64_t packet_time_s;
        uint64_t packet_time_ns;

        in_addr lidar_ip{};
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;
        std::string laserscan_topic;

        bool use_time_service{};
        bool coordinate_opt{};
        bool is_first_sweep;
        bool add_multicast{};
        double min_range{};
        double max_range{};
        double sweep_end_time;

        boost::shared_ptr<Input> msop_input_;
        boost::shared_ptr<Input> difop_input_;

        ros::Time last_publish_time;
        std::mutex laserscan_lock;
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        ros::Publisher scan_pub;

        unsigned char difop_data[1206]{};
        unsigned char packetTimeStamp[10];
        struct tm cur_time{};
        ros::Time timeStamp;
        ros::Time timeStamp_bak;
        double packet_rate;
        double current_packet_time;
        double last_packet_time;

        double horizontal_angle_resolution;
        double config_vertical_angle_32[32] = {0};
        double config_vertical_angle_tmp[32] = {0};
        double config_vertical_angle_16[16] = {0};

        std::atomic<bool> is_get_difop_;
        std::unique_ptr<ThreadPool> threadPool_;
        sensor_msgs::LaserScan::Ptr scan_msg;
        sensor_msgs::LaserScan::Ptr scan_msg_bak;
        uint point_size;
    };
}  // namespace lslidar_driver

#endif