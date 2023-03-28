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

#include "input.h"
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <lslidar_msgs/LslidarPacket.h>
#include <lslidar_msgs/LslidarPoint.h>
#include <lslidar_msgs/LslidarScan.h>
#include <lslidar_msgs/LslidarPacket.h>
#include <lslidar_msgs/lslidar_control.h>


namespace lslidar_driver {
//raw lslidar packet constants and structures
    static const int SIZE_BLOCK = 100;
    static const int RAW_SCAN_SIZE = 3;
    static const int SCANS_PER_BLOCK = 32;
    static const int BLOCK_DATA_SIZE = SCANS_PER_BLOCK * RAW_SCAN_SIZE;
    static const double DISTANCE_RESOLUTION = 0.01; //meters
    static const uint16_t UPPER_BANK = 0xeeff;

// special defines for lslidarlidar support
    static const int FIRING_TOFFSET = 32;
    static const int PACKET_SIZE = 1206;
    static const int BLOCKS_PER_PACKET = 12;
    static const int SCANS_PER_PACKET = SCANS_PER_BLOCK  * BLOCKS_PER_PACKET; //384
    //unit:meter
    static const double R1_ = 0.04319;
    static const double R2_ = 0.0494;
// Pre-compute the sine and cosine for the altitude angles.
    //1.33
    static const double c16_1_vertical_angle[16] = {-10, 0.665, -8.665, 2, -7.33, 3.33, -6, 4.665, -4.665, 6, -3.33,
                                                    7.33, -2, 8.665, -0.665, 10};
    //2°
    static const double c16_2_vertical_angle[16] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};


    // 0.33°
    static const double c32_1_vertical_angle[32] = {-18, -1, -15, -0.66, -12, -0.33, -10, 0, -8, 0.33, -7, 0.66, -6, 1,
                                                    -5, 1.33, -4, 1.66, -3.33, 2, -3, 3, -2.66, 4, -2.33, 6, -2, 8,
                                                    -1.66, 11, -1.33, 14};
    static const double c32_1_vertical_angle_26[32] = {-18, -15, -12, -10, -8, -7, -6, -5, -4, -3.33, -2.66, -3,
                                                       -2.33, -2, -1.33, -1.66, -1, -0.66, 0, -0.33, 0.33, 0.66,
                                                       1.33, 1, 1.66, 2, 3, 4, 6, 8, 11, 14};
    // 1°
    static const double c32_2_vertical_angle[32] = {-16, 0, -15, 1, -14, 2, -13, 3, -12, 4, -11, 5, -10, 6, -9, 7, -8,
                                                    8, -7, 9, -6, 10, -5, 11, -4, 12, -3, 13, -2, 14, -1, 15};

    static const double c32_2_vertical_angle_26[32] = {-16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4,
                                                       -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                                       15};
    static const uint8_t adjust_angle_index[32] = {0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8, 24, 9,
                                                   25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31};


/*    union TwoBytes {
        uint16_t distance;
        uint8_t bytes[2];
    };*/

    struct RawBlock {
        uint16_t header;
        uint16_t rotation;  //0-35999
        uint8_t data[BLOCK_DATA_SIZE];
    };

    struct RawPacket {
        RawBlock blocks[BLOCKS_PER_PACKET];
        uint32_t time_stamp;
        uint8_t factory[2];
    };

    struct Firing {
        uint16_t firing_azimuth[BLOCKS_PER_PACKET];
        int azimuth[SCANS_PER_PACKET];
        double distance[SCANS_PER_PACKET];
        double intensity[SCANS_PER_PACKET];
    };



    struct PointXYZIRT {
        PCL_ADD_POINT4D
        float intensity;
        std::uint16_t ring;
        float time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //make sure our new allocators are aligned
    }EIGEN_ALIGN16; //enforce SSE padding for correct memory alignment

    static std::string lidar_type;


    class lslidarDriver {
    public:
        lslidarDriver(ros::NodeHandle &node, ros::NodeHandle &private_nh);

        virtual ~lslidarDriver() {}

        bool checkPacketValidity(const RawPacket *packet);

        //check if a point is in the required range
        bool isPointInRange(const double &distance);

        bool loadParameters();

        void initTimeStamp();

        bool createRosIO();

        void publishPointcloud();

        void publishScan();


        bool lslidarC16Control(lslidar_msgs::lslidar_control::Request &req,
                               lslidar_msgs::lslidar_control::Response &res);

        bool SendPacketTolidar(bool power_switch);

        virtual void difopPoll() = 0;

        virtual bool poll() = 0;

        virtual bool initialize() = 0;

        virtual void decodePacket(const RawPacket *packet) = 0;


    public:
        int socket_id;
        int msop_udp_port;
        int difop_udp_port;
        int scan_num;
        int angle_disable_min;
        int angle_disable_max;
        uint16_t last_azimuth;
        uint16_t last_azimuth_tmp;
        uint64_t packet_time_s;
        uint64_t packet_time_ns;
        int return_mode;
        int c32_fpga_type;
        int config_vert_num;
        int count;
        int packet_size;

        in_addr lidar_ip;
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;
        std::string pointcloud_topic;
        std::string c32_type;
        std::string c16_type;

        bool use_gps_ts;
        bool pcl_type;
        bool publish_scan;
        bool coordinate_opt;
        bool is_first_sweep;
        bool add_multicast;
        bool config_vert;
        bool is_update_gps_time;
        double distance_unit;
        double min_range;
        double max_range;
        double sweep_end_time;
        double horizontal_angle_resolution;
        double packet_rate;
        //double packet_start_time;
        double packet_interval_time;
        double packet_timestamp;
        double last_packet_timestamp;

        double cos_azimuth_table[36000];
        double sin_azimuth_table[36000];


        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;
        std::shared_ptr<std::thread> difop_thread_;

        lslidar_msgs::LslidarScanPtr sweep_data;
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        //ros::Publisher packet_pub;
        ros::Publisher pointcloud_pub;
        ros::Publisher scan_pub;
        ros::ServiceServer lslidar_control;

        unsigned char difop_data[1206];
        unsigned char packetTimeStamp[10];
        struct tm cur_time;
        ros::Time timeStamp;
        ros::Time timeStamp_bak;
    };


    class lslidarC16Driver : public lslidarDriver {
    public:
        /**
       * @brief lslidarDriver
       * @param node          raw packet output topic
       * @param private_nh    通过这个节点传参数
       */
        lslidarC16Driver(ros::NodeHandle &node, ros::NodeHandle &private_nh);

        virtual ~lslidarC16Driver();

        virtual void difopPoll();


        virtual bool poll(void);


        virtual bool initialize();

        virtual void decodePacket(const RawPacket *packet);


    private:
        Firing firings;
        double c16_vertical_angle[16];
        double c16_config_vertical_angle[16];
        //double c16_scan_altitude[16];
        double c16_cos_scan_altitude[16];
        double c16_sin_scan_altitude[16];

    };


    class lslidarC32Driver : public lslidarDriver {
    public:
        lslidarC32Driver(ros::NodeHandle &node, ros::NodeHandle &private_nh);

        virtual ~lslidarC32Driver();

        virtual void difopPoll();

        virtual bool poll(void);

        virtual bool initialize();

        virtual void decodePacket(const RawPacket *packet);


    private:
        Firing firings;
        double c32_vertical_angle[32];
        double c32_config_vertical_angle[32];
        double c32_config_tmp_angle[32];
        // double c32_scan_altitude[32];
        double c32_cos_scan_altitude[32];
        double c32_sin_scan_altitude[32];
        int adjust_angle[4];

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
                                          (float, time, time))

#endif
