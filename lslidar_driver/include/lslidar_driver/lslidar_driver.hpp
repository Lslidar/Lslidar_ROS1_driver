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

#ifndef LSLIDAR_DRIVER_HPP
#define LSLIDAR_DRIVER_HPP

#define DEG_TO_RAD 0.017453f
#define RAD_TO_DEG 57.29577f

#include <ros/ros.h>
#include <string>
#include <memory>
#include <atomic>
#include <limits>
#include <functional>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "input.hpp"
#include "ThreadPool.h"
#include "lslidar_device_info.hpp"
#include "lslidar_pointcloud.hpp"
#include "pointcloud_transform.hpp"
#include "lslidar_driver/lslidar_services.hpp"
#include "lslidar_msgs/LslidarInformation.h"

namespace lslidar_driver {

    class LslidarDriver {
    public:
        explicit LslidarDriver(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
            : nh_(nh), private_nh_(private_nh) {}
        
        virtual ~LslidarDriver() = default;

        virtual bool loadParameters() = 0;

        virtual bool createRosIO() = 0;

        virtual void initTimeStamp() = 0;

        virtual bool initialize() = 0;

        virtual bool poll() = 0;
        
    protected:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Publisher pointcloud_pub_;
        ros::Publisher laserscan_pub_;
        ros::Publisher lidar_info_pub_;
        ros::Publisher time_pub_;
        
        ros::ServiceServer network_config_service_;
        ros::ServiceServer motor_speed_service_;
        ros::ServiceServer time_mode_service_;
        
        lslidar_msgs::LslidarInformationPtr lidar_info_data_;
        PointCloudTransform pointcloud_transform_;

        std::shared_ptr<Input> msop_input_;
        std::shared_ptr<Input> difop_input_;
        std::shared_ptr<LslidarServices> services_;
        std::shared_ptr<LidarDeviceInfo> device_info_ = std::make_shared<LidarDeviceInfo>();
        std::unique_ptr<ThreadPool> thread_pool_ = std::make_unique<ThreadPool>(3);

        std::atomic<bool> is_get_difop_{false};

        std::string lidar_model;
        std::string lidar_ip_string;
        std::string group_ip_string;
        std::string dump_file;
        std::string frame_id;
        std::string pointcloud_topic_;
        
        bool add_multicast;
        bool use_time_service;
        bool use_first_point_time;
        bool use_absolute_time;
        bool is_pretreatment;

        int msop_udp_port;
        int difop_udp_port;
        int point_time_offset;
        int relative_time_offset;

        double point_cloud_time;
        double last_point_cloud_time = 0.0;
        double packet_rate;
        double min_range;
        double max_range;

        double x_offset;
        double y_offset;
        double z_offset;
        double roll;
        double pitch;
        double yaw;
    };

}  // namespace lslidar_driver

#endif  // LSLIDAR_DRIVER_HPP

