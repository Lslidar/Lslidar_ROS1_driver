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

#include <ros/ros.h>
#include "lslidar_driver/lslidar_driver.hpp"
#include "lslidar_driver/lslidar_ch_driver.hpp"
#include "lslidar_driver/lslidar_cx_driver.hpp"
#include "lslidar_driver/lslidar_ls_driver.hpp"
#include "lslidar_driver/lslidar_x10_driver.hpp"

using namespace lslidar_driver;

volatile sig_atomic_t flag = 1;

static void my_handler(int sig) {
    flag = 0;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lslidar_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ROS_INFO("************ LSLiDAR ROS DRIVER VERSION: %s ************", lslidar_driver_VERSION);

    std::string lidar_type;
    private_nh.param<std::string>("lidar_type", lidar_type, "CX");

    std::shared_ptr<lslidar_driver::LslidarDriver> driver;

    try {
        if (lidar_type == "CX") {         // 机械式激光雷达
            driver = std::make_shared<lslidar_driver::LslidarCxDriver>(nh, private_nh);
        } else if (lidar_type == "CH"){   // 905激光雷达
            driver = std::make_shared<lslidar_driver::LslidarChDriver>(nh, private_nh);
        } else if (lidar_type == "LS") {  // 1550激光雷达
            driver = std::make_shared<lslidar_driver::LslidarLsDriver>(nh, private_nh);
        } else if (lidar_type == "X10") { // 单线激光雷达
            driver = std::make_shared<lslidar_driver::LslidarX10Driver>(nh, private_nh);
        } else {
            ROS_ERROR_STREAM("Invalid lidar type configured: '" << lidar_type 
                        << "'. Supported types are: CX, CH, LS, X10");
            throw std::invalid_argument("Unsupported lidar type");
        }

        if (!driver->initialize()) {
            throw std::runtime_error("Failed to initialize the Lslidar driver.");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error: %s", e.what());
        return -1;
    }

    std::unique_ptr<ThreadPool> threadPool = std::make_unique<ThreadPool>(1);
    threadPool->enqueue([&]() {
        while (ros::ok()) {
            driver->poll();
        }
    });

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();

    return 0;
}
