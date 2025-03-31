/******************************************************************************
 * This file is part of lslidar_driver.
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

#ifndef LSLIDAR_CX_DRIVER_HPP
#define LSLIDAR_CX_DRIVER_HPP

class Request;

#include "lslidar_driver/lslidar_driver.hpp"

#include <string>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/shared_ptr.hpp>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float64.h>
#include <yaml-cpp/yaml.h>
#include <regex>
#include <algorithm>
#include <cstdint>
#include <vector>

namespace lslidar_driver {
//raw lslidar packet constants and structures
    static const int SIZE_BLOCK = 100;
    static const int RAW_SCAN_SIZE = 3;
    static const int SCANS_PER_BLOCK = 32;
    static const int BLOCK_DATA_SIZE = SCANS_PER_BLOCK * RAW_SCAN_SIZE;
    static const float DISTANCE_RESOLUTION = 0.01f; //meters
    static const uint16_t UPPER_BANK = 0xeeff;

// special defines for lslidarlidar support
    static const int FIRINGS_PER_BLOCK = 2;
    static const int SCANS_PER_FIRING = 16;
    static const int SCANS_PER_FIRING_CX = 32;

    static const int FIRING_TOFFSET = 32;
    //  static const int PACKET_SIZE = 1212;
    static const int BLOCKS_PER_PACKET = 12;
    static const int FIRINGS_PER_PACKET_CX = BLOCKS_PER_PACKET;
    static const int SCANS_PER_PACKET = SCANS_PER_FIRING * FIRINGS_PER_BLOCK * BLOCKS_PER_PACKET; //384
    // 光学中心到结构中心距离
    static const float R1_ = 0.0361f;        // C16 4.0 or C32 4.0
    static const float R1_v5 = 0.035f;       // C16 5.0 or C32 5.0
    static const float R1_C32W = 0.03416f;   // 新C32W   byte[1211] = 0x47
    static const float R1_90 = 0.0209f;      // 90度
    static const double R2_ = 0.0431;        // C16 3.0
    static const double R3_ = 0.0494;        // C32 3.0 
    // 光学中心到结构中心角度
    static const int conversionAngle_ = 2025;   // C16 4.0 or C32 4.0
    static const int conversionAngle_v5 = 2180; // C16 5.0 or C32 5.0
    static const int conversionAngle_90 = 2776; // C32 90度
    static const int conversionAngle_C16_3 = 1468;  //C16 3.0
    static const int conversionAngle_C32_3 = 1298;  //C32 3.0

    static const double POINT_TIME_WEIGHT = 0.00260416;
    
    //c32 5.0 
    static const float c32_v5_vertical_angle[32] = {
            -20.8f, 0.0f, -10.4f, 10.4f, -15.6f, 5.2f, -5.2f, 15.6f,
            -19.5f, 1.3f, -9.1f, 11.7f, -14.3f, 6.5f, -3.9f, 16.9f,
            -18.2f, 2.6f, -7.8f, 13.0f, -13.0f, 7.8f, -2.6f, 18.2f,
            -16.9f, 3.9f, -6.5f, 14.3f, -11.7f, 9.1f, -1.3f, 19.5f};

    //c32 4.0 32度
    static const float c32_vertical_angle[32] = {
            -16.0f, -8.0f, 0.0f, 8.0f, -15.0f, -7.0f, 1.0f, 9.0f,
            -14.0f, -6.0f, 2.0f, 10.0f, -13.0f, -5.0f, 3.0f, 11.0f,
            -12.0f, -4.0f, 4.0f, 12.0f, -11.0f, -3.0f, 5.0f, 13.0f,
            -10.0f, -2.0f, 6.0f, 14.0f, -9.0f, -1.0f, 7.0f, 15.0f};

    //c32 70度  0x45    W_P
    static const float c32wp_vertical_angle[32] = {
            -51.0f, -31.0f, -9.0f, 3.0f, -48.5f, -28.0f, -7.5f, 4.5f,
            -46.0f, -25.0f, -6.0f, 6.0f, -43.5f, -22.0f, -4.5f, 7.5f,
            -41.0f, -18.5f, -3.0f, 9.0f, -38.5f, -15.0f, -1.5f, 11.0f,
            -36.0f, -12.0f, 0.0f, 13.0f, -33.5f, -10.5f, 1.5f, 15.0f};
    
    //c32 70度  0x46    w 
    static const float c32_70_vertical_angle[32] = {
            -54.0f, -31.0f, -8.0f, 2.66f, -51.5f, -28.0f, -6.66f, 4.0f,
            -49.0f, -25.0f, -5.33f, 5.5f, -46.0f, -22.0f, -4.0f, 7.0f,
            -43.0f, -18.5f, -2.66f, 9.0f, -40.0f, -15.0f, -1.33f, 11.0f,
            -37.0f, -12.0f, 0.0f, 13.0f, -34.0f, -10.0f, 1.33f, 15.0f};
    
    //c32 70度  0x47    wn wb
    static const float c32wn_vertical_angle2[32] = {
            -54.7f, -31.0f, -9.0f, 3.0f, -51.5f, -28.0f, -7.5f, 4.5f,
            -49.0f, -25.0f, -6.0f, 6.0f, -46.0f, -22.0f, -4.5f, 7.5f,
            -43.0f, -18.5f, -3.0f, 9.0f, -40.0f, -15.0f, -1.5f, 11.0f,
            -37.0f, -12.0f, 0.0f, 13.0f, -34.0f, -10.5f, 1.5f, 15.0f};

    //c32 70度  0x47    wn wb
    static const float c32wb_vertical_angle2[32] = {
            -54.7f, -9.0f, -31.0f, 3.0f, -51.5f, -7.5f, -28.0f, 4.5f,
            -49.0f, -6.0f, -25.0f, 6.0f, -46.0f, -4.5f, -22.0f, 7.5f,
            -43.0f, -3.0f, -18.5f, 9.0f, -40.0f, -1.5f, -15.0f, 11.0f,
            -37.0f, 0.0f, -12.0f, 13.0f, -34.0f, 1.5f, -10.5f, 15.0f};

    //c32 90度
    static const float c32_90_vertical_angle[32] = {
            2.487f, 25.174f, 47.201f, 68.819f, 5.596f, 27.811f, 49.999f, 71.525f,
            8.591f, 30.429f, 52.798f, 74.274f, 11.494f, 33.191f, 55.596f, 77.074f,
            14.324f, 36.008f, 58.26f, 79.938f, 17.096f, 38.808f, 60.87f, 82.884f,
            19.824f, 41.603f, 63.498f, 85.933f, 22.513f, 44.404f, 66.144f, 89.105f};

    static const float c32rn_vertical_angle[32] = {
            2.487f, 47.201f, 25.174f, 68.819f, 5.596f, 49.999f, 27.811f, 71.525f,
            8.591f, 52.798f, 30.429f, 74.274f, 11.494f, 55.596f, 33.191f, 77.074f,
            14.324f, 58.26f, 36.008f, 79.938f, 17.096f, 60.87f, 38.808f, 82.884f,
            19.824f, 63.498f, 41.603f, 85.933f, 22.513f, 66.144f, 44.404f, 89.105f};

    static float c16_vertical_angle[16] = {-16.0f, 0.0f, -14.0f, 2.0f, -12.0f, 4.0f, -10.0f, 6.0f, 
                                           -8.0f, 8.0f, -6.0f, 10.0f, -4.0f, 12.0f, -2.0f, 14.0f};

    static const float c16_domestic_vertical_angle[16] = {-16.0f, -8.0f, 0.0f, 8.0f, -14.0f, -6.0f, 2.0f, 10.0f, 
                                                          -12.0f, -4.0f, 4.0f, 12.0f, -10.0f, -2.0f, 6.0f, 14.0f};

    static const int cx_v5_remap_table[4] = {0, 2, 1, 3};

    static const float c16_v5_vertical_angle[16] = {-16.0f, 0.0f, -8.0f, 8.0f, -14.0f, 2.0f, -6.0f, 10.0f, 
                                                    -12.0f, 4.0f, -4.0f, 12.0f, -10.0f, 6.0f, -2.0f, 14.0f};
    
    static const int c16_remap_angle[16] = {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15};

    static const float c8_vertical_angle[8] = {-10.0f, 4.0f, -8.0f, 8.0f, -4.0f, 10.0f, 0.0f, 12.0f};

    static const float c8f_vertical_angle[8] = {-2.0f, 6.0f, 0.0f, 8.0f, 2.0f, 10.0f, 4.0f, 12.0f};
    
    static const float ckm8_vertical_angle[8] = {-12.0f, 4.0f, -8.0f, 8.0f, -4.0f, 10.0f, 0.0f, 12.0f};

    static const float c1_vertical_angle[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    //c16 3.0
    static const float c16_30_vertical_angle[16] = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};

    // c32 3.0
    static const float c32_30_vertical_angle[32] = {-16, 0, -15, 1, -14, 2, -13, 3, -12, 4, -11, 5, -10, 6, -9, 7, -8,
                                                   8, -7, 9, -6, 10, -5, 11, -4, 12, -3, 13, -2, 14, -1, 15};

    static const uint8_t adjust_angle_index[32] = {0, 16, 1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22, 7, 23, 8, 24, 9,
                                                   25, 10, 26, 11, 27, 12, 28, 13, 29, 14, 30, 15, 31};

    struct FiringCX {
        uint16_t firing_azimuth[FIRINGS_PER_PACKET_CX];
        int azimuth[SCANS_PER_PACKET];
        float distance[SCANS_PER_PACKET];
        float intensity[SCANS_PER_PACKET];
    };

    static std::string lidar_type;

    class LslidarCxDriver : public LslidarDriver {
    public:
        LslidarCxDriver(ros::NodeHandle &node, ros::NodeHandle &private_nh);

        virtual ~LslidarCxDriver() = default;
        
        bool loadParameters() override;

        bool createRosIO() override;

        void initTimeStamp() override;

        bool initialize() override;

        bool poll() override;

        void difopPoll();

        void publishPointcloud();

        void publishScan();

        bool initAngleConfig();

        bool checkPacketValidity(lslidar_msgs::LslidarPacketPtr &packet);

        void decodePacket(lslidar_msgs::LslidarPacketPtr &packet);

        int calculateRing(size_t fir_idx) const;

        bool isPointValid(const int fir_idx, const int remapped_scan_idx) const;

        void pointcloudToLaserscan(const sensor_msgs::PointCloud2 &cloud_msg, sensor_msgs::LaserScan &output_scan);

        bool determineLidarType();

    public:
        int scan_num{};
        int point_num{};
        int angle_disable_min{};
        int angle_disable_max{};
        int angle_able_min{};
        int angle_able_max{};

        uint16_t last_azimuth;
        uint64_t packet_time_s;
        uint64_t packet_time_ns;
        int return_mode;
        int fpga_type{};

        in_addr lidar_ip{};
        std::string filter_angle_file;

        bool pcl_type{};
        bool publish_scan{};
        bool is_first_sweep;
        bool angle_change{};
        float distance_unit = 0.40;

        double sweep_end_time;
        float cos_azimuth_table[36000]{};
        float sin_azimuth_table[36000]{};
        float R1 = 0.0f;

        std::mutex pointcloud_lock;
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        ros::ServiceServer motor_control_service;
        ros::ServiceServer power_control_service;
        ros::ServiceServer rfd_removal_service;
        ros::ServiceServer tail_removal_service;

        unsigned char packetTimeStamp[10];
        struct tm cur_time{};
        ros::Time timeStamp;
        ros::Time timeStamp_bak;
        double current_packet_time;
        double last_packet_time;

        FiringCX firings{};
        float scan_altitude[32] = {0.0f};
        float cos_scan_altitude[32] = {0.0f};
        float sin_scan_altitude[32] = {0.0f};
        float msc16_adjust_angle[16] = {0.0f};
        float msc16_offset_angle[16] = {0.0f};

        double horizontal_angle_resolution;
        int adjust_angle[4];
        int config_vert_num;
        double config_vertical_angle_32[32] = {0};
        double config_vertical_angle_tmp[32] = {0};
        double config_vertical_angle_16[16] = {0};

        int lidar_number_;
        int ring_;
        std::atomic<int> time_service_mode{0};
        int remove_rain_fog_dust;      // 去雨雾尘
        int remove_mirror_points;      // 去除玻璃镜像
        int remove_tail_points;        // 去除拖尾档位
        int tail_filter_distance;      // 滤拖尾距离

        bool start_process_msop;
        bool is_msc16;

        int conversionAngle{};
        int config_vertical_angle_flag;
        int filter_angle[32][2];

        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_bak_;
        sensor_msgs::LaserScan::Ptr scan_msg;
        sensor_msgs::LaserScan::Ptr scan_msg_bak;
        uint point_size;
        int packet_num{};
        int number_threshold = 10;
    };

    typedef PointXYZIRT VPoint;
    typedef pcl::PointCloud<VPoint> VPointcloud;
}  // namespace lslidar_driver

#endif // LSLIDAR_CX_DRIVER_HPP
