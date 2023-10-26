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

#define DEG_TO_RAD 0.017453f
#define RAD_TO_DEG 57.29577f

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
#include <regex>
#include <atomic>
#include <thread>
#include <memory>
#include <mutex>
#include <algorithm>
#include "lslidar_cx_driver/lslidar_control.h"
#include "lslidar_cx_driver/time_service.h"
#include "lslidar_cx_driver/motor_speed.h"
#include "lslidar_cx_driver/motor_control.h"
#include "lslidar_cx_driver/remove_control.h"
#include "lslidar_cx_driver/data_port.h"
#include "lslidar_cx_driver/dev_port.h"
#include "lslidar_cx_driver/data_ip.h"
#include "lslidar_cx_driver/destination_ip.h"


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
    //unit:meter
    static const float R1_ = 0.0361f;        // C16 4.0 or C32 4.0
    static const float R1_C32W = 0.03416f;   // 新C32W   byte[1211] = 0x47
    static const float R1_90 = 0.0209f;   // 90度
    static const int conversionAngle_ = 2025;
    static const int conversionAngle_90 = 2776; // 90度
// Pre-compute the sine and cosine for the altitude angles.
    //c32 32度
    static const float c32_vertical_angle[32] = {-16.0f, -8.0f, 0.0f, 8.0f, -15.0f, -7.0f, 1.0f, 9.0f,
                                                 -14.0f, -6.0f, 2.0f, 10.0f, -13.0f, -5.0f, 3.0f, 11.0f,
                                                 -12.0f, -4.0f, 4.0f, 12.0f, -11.0f, -3.0f, 5.0f, 13.0f,
                                                 -10.0f, -2.0f, 6.0f, 14.0f, -9.0f, -1.0f, 7.0f, 15.0f};

    //c32 70度  0x45
    static const float c32wa_vertical_angle[32] = {
            -51.0f, -31.0f, -9.0f, 3.0f, -48.5f, -28.0f, -7.5f, 4.5f,
            -46.0f, -25.0f, -6.0f, 6.0f, -43.5f, -22.0f, -4.5f, 7.5f,
            -41.0f, -18.5f, -3.0f, 9.0f, -38.5f, -15.0f, -1.5f, 11.0f,
            -36.0f, -12.0f, 0.0f, 13.0f, -33.5f, -10.5f, 1.5f, 15.0f};

    //c32 70度  0x46
    static const float c32_70_vertical_angle[32] = {
            -54.0f, -31.0f, -8.0f, 2.66f, -51.5f, -28.0f, -6.66f, 4.0f,
            -49.0f, -25.0f, -5.33f, 5.5f, -46.0f, -22.0f, -4.0f, 7.0f,
            -43.0f, -18.5f, -2.66f, 9.0f, -40.0f, -15.0f, -1.33f, 11.0f,
            -37.0f, -12.0f, 0.0f, 13.0f, -34.0f, -10.0f, 1.33f, 15.0f};

    //c32 70度  0x47
    static const float c32_70_vertical_angle2[32] = {
            -54.7f, -31.0f, -9.0f, 3.0f, -51.5f, -28.0f, -7.5f, 4.5f,
            -49.0f, -25.0f, -6.0f, 6.0f, -46.0f, -22.0f, -4.5f, 7.5f,
            -43.0f, -18.5f, -3.0f, 9.0f, -40.0f, -15.0f, -1.5f, 11.0f,
            -37.0f, -12.0f, 0.0f, 13.0f, -34.0f, -10.5f, 1.5f, 15.0f};


    //c32 90度
    static const float c32_90_vertical_angle[32] = {
            2.487f, 25.174f, 47.201f, 68.819f, 5.596f, 27.811f, 49.999f, 71.525f,
            8.591f, 30.429f, 52.798f, 74.274f, 11.494f, 33.191f, 55.596f, 77.074f,
            14.324f, 36.008f, 58.26f, 79.938f, 17.096f, 38.808f, 60.87f, 82.884f,
            19.824f, 41.603f, 63.498f, 85.933f, 22.513f, 44.404f, 66.144f, 89.105f};

//    static const double c32_70_vertical_angle[32] = {-55, -30, -8, 2.66, -51.5, -27, -6.65, 3.99, -48, -24, -5.32, 5.5,
//                                                     -45, -21, -3.99, 7, -42, -18, -2.66, 9, -39, -15, -1.33, 11, -36,
//                                                     -12.5, 0, 13, -33, -10, 1.33, 15};
//2°
    static float c16_vertical_angle[16] = {-16.0f, 0.0f, -14.0f, 2.0f, -12.0f, 4.0f, -10.0f, 6.0f, -8.0f, 8.0f,
                                           -6.0f, 10.0f, -4.0f, 12.0f, -2.0f, 14.0f};
    static const int c16_remap_angle[16] = {0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15};
    // static const double c8_vertical_angle[8] = {-12, 0, -10, 4, -8, 8, -4, 10};
    //static const double c8_vertical_angle[8] = {-10, 4, -8, 8, -4, 10, 0, 12};
    static const float c8_vertical_angle[8] = {-12.0f, 4.0f, -8.0f, 8.0f, -4.0f, 10.0f, 0.0f, 12.0f};

    //static const float c8f_vertical_angle[8] = {-2.0f, 0.0f, 2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 12.0f};
    static const float c8f_vertical_angle[8] = {-2.0f, 6.0f, 0.0f, 8.0f, 2.0f, 10.0f, 4.0f, 12.0f};
    // static const float c8s_vertical_angle[8] = {-12.0f, -8.0f, -4.0f, 0.0f, 4.0f, 8.0f, 10.0f, 12.0f};
    static const float c8s_vertical_angle[8] = {-12.0f, 4.0f, -8.0f, 8.0f, -4.0f, 10.0f, 0.0f, 12.0f};

    static const float c1_vertical_angle[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};


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
        float distance[SCANS_PER_PACKET];
        float intensity[SCANS_PER_PACKET];
    };

    struct PointXYZIRT {
        float x;
        float y;
        float z;
        float intensity;
        std::uint16_t ring;
        float time;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW //make sure our new allocators are aligned
    }EIGEN_ALIGN16; //enforce SSE padding for correct memory alignment

    static std::string lidar_type;

    class LslidarDriver {
    public:
        LslidarDriver(ros::NodeHandle &node, ros::NodeHandle &private_nh);

        ~LslidarDriver() {}

        bool checkPacketValidity(lslidar_cx_driver::LslidarPacketPtr &packet);

        //check if a point is in the required range
        bool isPointInRange(const double &distance);

        bool loadParameters();

        void initTimeStamp();

        bool createRosIO();

        void difopPoll();

        void publishPointcloud();

        void publishScan();

        bool powerOn(lslidar_cx_driver::lslidar_control::Request &req,
                     lslidar_cx_driver::lslidar_control::Response &res);

        bool motorControl(lslidar_cx_driver::motor_control::Request &req,
                          lslidar_cx_driver::motor_control::Response &res);

        bool removeControl(lslidar_cx_driver::remove_control::Request &req,
                          lslidar_cx_driver::remove_control::Response &res);

        bool motorSpeed(lslidar_cx_driver::motor_speed::Request &req,
                        lslidar_cx_driver::motor_speed::Response &res);

        bool timeService(lslidar_cx_driver::time_service::Request &req,
                         lslidar_cx_driver::time_service::Response &res);

        bool setDataPort(lslidar_cx_driver::data_port::Request &req,
                         lslidar_cx_driver::data_port::Response &res);

        bool setDevPort(lslidar_cx_driver::dev_port::Request &req,
                        lslidar_cx_driver::dev_port::Response &res);

        bool setDataIp(lslidar_cx_driver::data_ip::Request &req,
                       lslidar_cx_driver::data_ip::Response &res);

        bool setDestinationIp(lslidar_cx_driver::destination_ip::Request &req,
                              lslidar_cx_driver::destination_ip::Response &res);

        static void setPacketHeader(unsigned char *config_data);

        bool sendPacketTolidar(unsigned char *config_data) const;

        //void decodePacket(const RawPacket* &packet);
        void decodePacket(lslidar_cx_driver::LslidarPacketPtr &packet);

        bool poll();

        void pointcloudToLaserscan(const sensor_msgs::PointCloud2 &cloud_msg, sensor_msgs::LaserScan &output_scan);

        bool initialize();

    public:
        int msop_udp_port{};
        int difop_udp_port{};
        int scan_num{};
        int point_num{};
        int angle_disable_min{};
        int angle_disable_max{};
        uint16_t last_azimuth;
        uint64_t packet_time_s{};
        uint64_t packet_time_ns{};
        int return_mode;

        in_addr lidar_ip{};
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string frame_id;
        std::string dump_file;
        std::string pointcloud_topic;

        bool use_time_service{};
        bool pcl_type{};
        bool publish_scan{};
        bool coordinate_opt{};
        bool is_first_sweep;
        bool add_multicast{};
        // std::string c32_type;
        double distance_unit{};
        double min_range{};
        double max_range{};
        double sweep_end_time;
        double angle_base{};
        float cos_azimuth_table[36000]{};
        float sin_azimuth_table[36000]{};

        boost::shared_ptr<Input> msop_input_;
        boost::shared_ptr<Input> difop_input_;
        boost::shared_ptr<boost::thread> difop_thread_;

        //lslidar_driver::LslidarScanPtr sweep_data;
        //lslidar_driver::LslidarScanPtr sweep_data_bak;
        std::mutex pointcloud_lock;
        ros::NodeHandle nh;
        ros::NodeHandle pnh;
        //ros::Publisher packet_pub;
        ros::Publisher pointcloud_pub;
        ros::Publisher scan_pub;
        ros::ServiceServer time_service_;               //授时
        ros::ServiceServer lslidar_control_service_;    //上下电
        ros::ServiceServer motor_control_service_;      //雷达转动/停转
        ros::ServiceServer remove_control_service_;      //雷达去除雨雾尘级别
        ros::ServiceServer motor_speed_service_;        //雷达频率
        ros::ServiceServer data_port_service_;          //数据包端口
        ros::ServiceServer dev_port_service_;           //设备包端口
        ros::ServiceServer data_ip_service_;            //数据包ip
        ros::ServiceServer destination_ip_service_;     //设备包ip

        unsigned char difop_data[1206]{};
        unsigned char packetTimeStamp[10]{};
        struct tm cur_time{};
        ros::Time timeStamp;
        double packet_rate;
        double current_packet_time;
        double last_packet_time;
        double current_point_time;
        double last_point_time;
        FiringCX firings{};
        float scan_altitude[32] = {0.0f};
        float cos_scan_altitude[32] = {0.0f};
        float sin_scan_altitude[32] = {0.0f};
        float msc16_adjust_angle[16] = {0.0f};
        float msc16_offset_angle[16] = {0.0f};
        double horizontal_angle_resolution;
        int lidar_number_;
        std::atomic<bool> is_get_difop_{false};
        std::atomic<int> time_service_mode_{0};
        int remove_rain_flag;
        bool start_process_msop_;
        bool is_new_c32w_;
        bool is_get_scan_altitude_;
        bool is_msc16;
        int config_vertical_angle_flag;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_xyzi_bak_;
        sensor_msgs::LaserScan::Ptr scan_msg;
        sensor_msgs::LaserScan::Ptr scan_msg_bak;
        uint point_size;

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
