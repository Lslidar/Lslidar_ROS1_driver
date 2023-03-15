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
#ifndef _LS_C16_DRIVER_H_
#define _LS_C16_DRIVER_H_

#define DEG_TO_RAD 0.017453293
#define RAD_TO_DEG 57.29577951

class Request;

#include "input.h"
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <lslidar_msgs/LslidarPacket.h>
#include <lslidar_msgs/LslidarPoint.h>
#include <lslidar_msgs/LslidarScan.h>
#include <lslidar_msgs/LslidarC16Sweep.h>
#include <lslidar_msgs/LslidarC32Sweep.h>
#include <lslidar_msgs/LslidarScanUnified.h>
#include <lslidar_msgs/LslidarPacket.h>
#include <lslidar_msgs/lslidar_control.h>
#include <lslidar_msgs/time_service.h>
#include <lslidar_msgs/motor_speed.h>
#include <lslidar_msgs/motor_control.h>
#include <lslidar_msgs/data_port.h>
#include <lslidar_msgs/dev_port.h>
#include <lslidar_msgs/data_ip.h>
#include <lslidar_msgs/destination_ip.h>
#include <regex>
#include <atomic>

#include <memory>
#include <mutex>


namespace lslidar_driver {
//raw lslidar packet constants and structures
    static const int SIZE_BLOCK = 100;
    static const int RAW_SCAN_SIZE = 3;
    static const int SCANS_PER_BLOCK = 32;
    static const int BLOCK_DATA_SIZE = SCANS_PER_BLOCK * RAW_SCAN_SIZE;
    static const double DISTANCE_RESOLUTION = 0.01; //meters
    static const uint16_t UPPER_BANK = 0xeeff;

// special defines for lslidarlidar support
    static const int FIRINGS_PER_BLOCK = 2;
    static const int SCANS_PER_FIRING = 16;
    static const int SCANS_PER_FIRING_CX = 32;
    static const int DSR_TOFFSET = 1;
    static const int FIRING_TOFFSET = 16;
    static const int FIRING_TOFFSET_C32 = 32;
    static const int FIRING_TOFFSET_C8 = 32;
    static const int PACKET_SIZE = 1212;
    static const int BLOCKS_PER_PACKET = 12;
    static const int FIRINGS_PER_PACKET_CX = BLOCKS_PER_PACKET;
    static const int SCANS_PER_PACKET = SCANS_PER_FIRING * FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET; //384
    //unit:meter
    static const double R1_ = 0.0361;     //
    static const double R1_90 = 0.0209;   // 90度
    static const double conversionAngle_ = 20.25;
    static const double conversionAngle_90 = 27.76; // 90度
// Pre-compute the sine and cosine for the altitude angles.
    //c32 32度
    static const double c32_vertical_angle[32] = {-16, -8, 0, 8, -15, -7, 1, 9, -14, -6, 2, 10, -13, -5, 3, 11, -12,
                                                  -4, 4, 12, -11, -3, 5, 13, -10, -2, 6, 14, -9, -1, 7, 15};
    //c32 70度
    static const double c32_70_vertical_angle[32] = {
            -54, -31, -8, 2.66, -51.5, -28, -6.66, 4,
            -49, -25, -5.33, 5.5, -46, -22, -4, 7,
            -43, -18.5, -2.66, 9, -40, -15, -1.33, 11,
            -37, -12, 0, 13, -34, -10, 1.33, 15};

    //c32 90度
    static const double c32_90_vertical_angle[32] = {
            2.487, 25.174, 47.201, 68.819, 5.596, 27.811, 49.999, 71.525,
            8.591, 30.429, 52.798, 74.274, 11.494, 33.191, 55.596, 77.074,
            14.324, 36.008, 58.26, 79.938, 17.096, 38.808, 60.87, 82.884,
            19.824, 41.603, 63.498, 85.933, 22.513, 44.404, 66.144, 89.105};

//    static const double c32_70_vertical_angle[32] = {-55, -30, -8, 2.66, -51.5, -27, -6.65, 3.99, -48, -24, -5.32, 5.5,
//                                                     -45, -21, -3.99, 7, -42, -18, -2.66, 9, -39, -15, -1.33, 11, -36,
//                                                     -12.5, 0, 13, -33, -10, 1.33, 15};
//2°
    static const double c16_vertical_angle[16] = {-16, 0, -14, 2, -12, 4, -10, 6, -8, 8, -6, 10, -4, 12, -2, 14};

    // static const double c8_vertical_angle[8] = {-12, 0, -10, 4, -8, 8, -4, 10};
    //static const double c8_vertical_angle[8] = {-10, 4, -8, 8, -4, 10, 0, 12};
    static const double c8_vertical_angle[8] = {-12, 4, -8, 8, -4, 10, 0, 12};
    static const double c1_vertical_angle[8] = {0, 0, 0, 0, 0, 0, 0, 0};


    union TwoBytes {
        uint16_t distance;
        uint8_t bytes[2];
    };

    struct RawBlock {
        uint16_t header;
        uint16_t rotation;  //0-35999
        uint8_t data[BLOCK_DATA_SIZE];
    };

    struct RawPacket {
        RawBlock blocks[BLOCKS_PER_PACKET];
        uint8_t time_stamp[10];
        uint8_t factory[2];
    };

    struct FiringCX {
        uint16_t firing_azimuth[FIRINGS_PER_PACKET_CX];
        uint16_t azimuth[SCANS_PER_PACKET];
        double distance[SCANS_PER_PACKET];
        double intensity[SCANS_PER_PACKET];
    };

    struct PointXYZIRT {
        PCL_ADD_POINT4D;
        PCL_ADD_INTENSITY;
        std::uint16_t ring;
        double time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //make sure our new allocators are aligned
    }EIGEN_ALIGN16; //enforce SSE padding for correct memory alignment

    static std::string lidar_type;

    class LslidarDriver {
    public:
        LslidarDriver(ros::NodeHandle &node, ros::NodeHandle &private_nh);

        ~LslidarDriver() {}

        bool checkPacketValidity(const RawPacket *packet);

        //check if a point is in the required range
        bool isPointInRange(const double &distance);

        bool loadParameters();

        void initTimeStamp();

        bool createRosIO();

        void difopPoll();

        void publishPointcloud();

        void publishScan();

        bool powerOn(lslidar_msgs::lslidar_control::Request &req,
                               lslidar_msgs::lslidar_control::Response &res);

        bool motorControl(lslidar_msgs::motor_control::Request &req,
                     lslidar_msgs::motor_control::Response &res);

        bool motorSpeed(lslidar_msgs::motor_speed::Request &req,
                          lslidar_msgs::motor_speed::Response &res);

        bool timeService(lslidar_msgs::time_service::Request &req,
                               lslidar_msgs::time_service::Response &res);

        bool setDataPort(lslidar_msgs::data_port::Request &req,
                        lslidar_msgs::data_port::Response &res);

        bool setDevPort(lslidar_msgs::dev_port::Request &req,
                         lslidar_msgs::dev_port::Response &res);

        bool setDataIp(lslidar_msgs::data_ip::Request &req,
                         lslidar_msgs::data_ip::Response &res);

        bool setDestinationIp(lslidar_msgs::destination_ip::Request &req,
                        lslidar_msgs::destination_ip::Response &res);

        static void setPacketHeader(unsigned char *config_data);

        bool sendPacketTolidar(unsigned char *config_data) const;

        void decodePacket(const RawPacket *packet);

        bool poll();

        void pointcloudToLaserscan(const sensor_msgs::PointCloud2& cloud_msg,sensor_msgs::LaserScan &output_scan);

        bool initialize();

    public:
        int msop_udp_port;
        int difop_udp_port;
        int scan_num;
        int point_num;
        int angle_disable_min;
        int angle_disable_max;
        uint16_t last_azimuth;
        uint64_t packet_time_s;
        uint64_t packet_time_ns;
        int return_mode;

        in_addr lidar_ip;
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;
        std::string pointcloud_topic;

        bool use_gps_ts;
        bool pcl_type;
        bool publish_scan;
        bool coordinate_opt;
        bool is_first_sweep;
        bool add_multicast;
        std::string c32_type;
        double distance_unit;
        double min_range;
        double max_range;
        double sweep_end_time;
        double angle_base;
        double cos_azimuth_table[36000];
        double sin_azimuth_table[36000];

        boost::shared_ptr<Input> msop_input_;
        boost::shared_ptr<Input> difop_input_;
        boost::shared_ptr<boost::thread> difop_thread_;

        lslidar_msgs::LslidarScanPtr sweep_data;
        lslidar_msgs::LslidarScanPtr sweep_data_bak;
        std::mutex pointcloud_lock;
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        //ros::Publisher packet_pub;
        ros::Publisher pointcloud_pub;
        ros::Publisher scan_pub;
        ros::ServiceServer time_service_;               //授时
        ros::ServiceServer lslidar_control_service_;    //上下电
        ros::ServiceServer motor_control_service_;      //雷达转动/停转
        ros::ServiceServer motor_speed_service_;        //雷达频率
        ros::ServiceServer data_port_service_;          //数据包端口
        ros::ServiceServer dev_port_service_;           //设备包端口
        ros::ServiceServer data_ip_service_;            //数据包ip
        ros::ServiceServer destination_ip_service_;     //设备包ip

        unsigned char difop_data[1206];
        unsigned char packetTimeStamp[10];
        struct tm cur_time;
        ros::Time timeStamp;
        double packet_rate;
        double current_packet_time;
        double last_packet_time;
        double current_point_time;
        double last_point_time;
        FiringCX firings;
        double scan_altitude[32] = {0};
        double cos_scan_altitude[32] = {0};
        double sin_scan_altitude[32] = {0};
        double horizontal_angle_resolution;
        int lidar_number_ ;
        std::atomic<bool> is_get_difop_ {false};
        std::atomic<int> time_service_mode_{0};
    };

    typedef PointXYZIRT VPoint;
    typedef pcl::PointCloud<VPoint> VPointcloud;

}  // namespace lslidar_driver

POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_driver::PointXYZIRT,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (double, time, time))

#endif
