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

#ifndef LSLIDAR_Ls_DRIVER_HPP
#define LSLIDAR_Ls_DRIVER_HPP

#include "lslidar_driver/lslidar_driver.hpp"

#include <regex>
#include <cmath>
#include <chrono>
#include <deque>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/register_point_struct.h>

namespace lslidar_driver {
    /** Special Defines for LSCh support **/
    constexpr int POINTS_PER_PACKET_SINGLE_ECHO = 1192;       // modify
    constexpr int POINTS_PER_PACKET_DOUBLE_ECHO = 1188;       // modify
    constexpr double SINGLE_ECHO = 0.006711409;
    constexpr double DOUBLE_ECHO = 0.01010101;

    static float g_fDistanceAcc = 0.001f;
    static float m_offset = 6.37f;
    constexpr double cos30 = cos(DEG2RAD(30));
    constexpr double sin30 = sin(DEG2RAD(30));
    constexpr double cos60 = cos(DEG2RAD(60));
    constexpr double sin60 = sin(DEG2RAD(60));

    struct FiringLS {
        double vertical_angle;
        double azimuth;
        double distance;
        float intensity;
        float time;
        int channel_number;
    };

    class LslidarLsDriver : public LslidarDriver {
    public:
        LslidarLsDriver(ros::NodeHandle &node, ros::NodeHandle &private_nh);

        virtual ~LslidarLsDriver() = default;

        bool loadParameters() override;

        bool createRosIO() override;

        void initTimeStamp() override;

        bool initialize() override;

        bool poll() override;

        void difopPoll();

        std::function<int(const struct FiringLS &)> lidarConvertCoordinate;

        inline int convertCoordinate(const struct FiringLS &lidardata);

        inline int convertCoordinateDistortion(const struct FiringLS &lidardata);

        void publishPointCloudNew();

        static void setPacketHeader(unsigned char *config_data);

        bool sendPacketTolidar(unsigned char *config_data) const;

        std::function<void(const lslidar_msgs::LslidarPacketPtr&)> lslidarPacketProcess;

        void packetProcessSingle(const lslidar_msgs::LslidarPacketPtr& packet);
        
        void packetProcessDouble(const lslidar_msgs::LslidarPacketPtr& packet);

        void checkPacketLoss(const lslidar_msgs::LslidarPacketPtr &msg, int data_offset, int byte_count);

        void updateTimeOffsets(double point_interval_time,int point_size);

        bool getLidarInformation();

        Information getDeviceInfo(const lslidar_msgs::LslidarPacketPtr &packet);

        typedef boost::shared_ptr<LslidarLsDriver> LslidarLsDriverPtr;
        typedef boost::shared_ptr<const LslidarLsDriver> LslidarLsDriverConstPtr;

    public:
        in_addr lidar_ip{};
        double prism_angle[4]{};

        // ROS related variables
        ros::NodeHandle nh;
        ros::NodeHandle pnh;

        ros::Publisher packet_loss_pub;
        ros::Publisher fault_code_pub;

        ros::ServiceServer angle_distortion_correction_service;
        ros::ServiceServer frame_rate_service;
        ros::ServiceServer invalid_data_service;
        ros::ServiceServer standby_mode_service;

        uint64_t pointcloudTimeStamp{};
        unsigned char packetTimeStamp[10]{};

        ros::Time packet_timeStamp;
        double packet_end_time;
        double current_packet_time;
        double last_packet_time;
        double packet_interval_time;
        double point_cloud_timestamp;
        
        int return_mode;
        int scan_start_angle{};
        int scan_end_angle{};
        double g_fAngleAcc_V;
        bool is_add_frame_;
        bool packet_loss;
        std::mutex pc_mutex_;
        
        int64_t last_packet_number_;
        int64_t tmp_packet_number_;
        int64_t total_packet_loss_;
        int frame_count;
        int m_horizontal_point = -1;
        bool get_ms06_param;

        double cos_table[36000]{};
        double sin_table[36000]{};
        double cos_mirror_angle[4]{};
        double sin_mirror_angle[4]{};

        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_bak_;
        pcl::PointCloud<PointXYZIRT>::Ptr point_cloud_xyzirt_pub_;
    };
    typedef PointXYZIRT VPoint;
    typedef LslidarLsDriver::LslidarLsDriverPtr LslidarLsDriverPtr;
    typedef LslidarLsDriver::LslidarLsDriverConstPtr LslidarLsDriverConstPtr;
} // namespace lslidar_driver

#endif // _LSLIDAR_Ls_DRIVER_HPP