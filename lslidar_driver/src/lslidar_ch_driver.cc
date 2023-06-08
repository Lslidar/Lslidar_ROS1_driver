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
#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <lslidar_ch_driver/lslidar_ch_driver.h>

namespace lslidar_ch_driver {

    LslidarChDriver::LslidarChDriver(
            ros::NodeHandle &n, ros::NodeHandle &pn) :
            nh(n),
            pnh(pn),
            socket_id(-1),
            sweep_data(new lslidar_msgs::LslidarChScan()),
            sweep_data_bac(new lslidar_msgs::LslidarChScan()) {
        first_frame_flag = 0;
        last_packet_timestamp = 0.0;
        packetType = false;
        is_update_difop = false;

        return;
    }

    LslidarChDriver::~LslidarChDriver() {

        if (nullptr == difop_thread_) {
            difop_thread_->join();
        }
        return;
    }

    bool LslidarChDriver::loadParameters() {

        pnh.param("frame_id", frame_id, std::string("laser_link"));
        pnh.param("lidar_ip", lidar_ip_string, std::string("192.168.1.200"));
        pnh.param<bool>("add_multicast", add_multicast, false);
        pnh.param<bool>("pcl_type", pcl_type, false);
        pnh.param<std::string>("lidar_type", lidar_type, "ch32");
        pnh.param("group_ip", group_ip_string, std::string("224.1.1.2"));
        pnh.param("msop_port", msop_udp_port, (int) MSOP_DATA_PORT_NUMBER);
        pnh.param("difop_port", difop_udp_port, (int) DIFOP_DATA_PORT_NUMBER);
        pnh.param("min_range", min_range, 0.5);
        pnh.param("max_range", max_range, 150.0);
        pnh.param<double>("angle_disable_min", angle_disable_min, 0.0);
        pnh.param<double>("angle_disable_max", angle_disable_max, 0.0);
        pnh.param<double>("horizontal_angle_resolution", horizontal_angle_resolution, 0.2);
        pnh.param<std::string>("pointcloud_topic", pointcloud_topic, "lslidar_point_cloud");
        pnh.param<bool>("publish_laserscan", publish_laserscan, false);
        pnh.param<int>("channel_num", channel_num, 8);
        pnh.param<int>("echo_num", echo_num, 0);
        pnh.param<int>("field_angle_type", field_angle_type, 1);
        pnh.param("pcap", dump_file, std::string(""));
        pnh.param("packet_rate", packet_rate, 5050.0);
        pnh.param("use_time_service", use_time_service, false);

        ROS_INFO("using time service or not: %d", use_time_service);
        ROS_INFO("lidar type: %s", lidar_type.c_str());
        inet_aton(lidar_ip_string.c_str(), &lidar_ip);

        if (add_multicast) ROS_INFO_STREAM("Opening UDP socket: group_address " << group_ip_string);


        if (publish_laserscan) {
            if (channel_num < 0) {
                channel_num = 0;
                ROS_WARN("channel_num outside of the index, select channel 0 instead!");
            } else if (channel_num > 119) {
                channel_num = 119;
                ROS_WARN("channel_num outside of the index, select channel 119 instead!");
            }
            ROS_INFO("select channel num: %d", channel_num);
        }

/*        switch (frequency) {
            case 5:
                horizontal_angle_resolution = DEG2RAD(0.1);
                break;
            case 20:
                horizontal_angle_resolution = DEG2RAD(0.4);
                break;
            default:
                horizontal_angle_resolution = DEG2RAD(0.2);
        }*/


        return true;
    }

    bool LslidarChDriver::createRosIO() {

        // ROS diagnostics

        pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 10);
        laserscan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);

        if (dump_file != "") {
            msop_input_.reset(new lslidar_ch_driver::InputPCAP(pnh, msop_udp_port, packet_rate, dump_file));
            difop_input_.reset(new lslidar_ch_driver::InputPCAP(pnh, difop_udp_port, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_ch_driver::InputSocket(pnh, msop_udp_port));
            difop_input_.reset(new lslidar_ch_driver::InputSocket(pnh, difop_udp_port));
        }

        difop_thread_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&LslidarChDriver::difopPoll, this)));


        return true;
    }

    bool LslidarChDriver::initialize() {

        this->initTimeStamp();

        if (!loadParameters()) {
            ROS_ERROR("Cannot load all required ROS parameters...");
            return false;
        }

        if (!createRosIO()) {
            ROS_ERROR("Cannot create all ROS IO...");
            return false;
        }
        for (int j = 0; j < 36000; ++j) {
            sin_azimuth_table[j] = sin(j * 0.01 * 0.017453293);
            cos_azimuth_table[j] = cos(j * 0.01 * 0.017453293);
        }
        for (int i = 0; i < 120; i = i + 6) {
            theta_t[i] = sin((-13 + i * 0.167) * DEG_TO_RAD);
            theta_t[i + 1] = sin((-13 + i * 0.167) * DEG_TO_RAD);
            theta_t[i + 2] = sin((-13 + i * 0.167) * DEG_TO_RAD);
            theta_t[i + 3] = sin((-13 + i * 0.167) * DEG_TO_RAD);
            theta_t[i + 4] = sin((-13 + i * 0.167) * DEG_TO_RAD);
            theta_t[i + 5] = sin((-13 + i * 0.167) * DEG_TO_RAD);

            theta_q[i] = 0;
            theta_q[i + 1] = sin(0.12 * DEG_TO_RAD);
            theta_q[i + 2] = sin(0.24 * DEG_TO_RAD);
            theta_q[i + 3] = sin(0.36 * DEG_TO_RAD);
            theta_q[i + 4] = sin(0.48 * DEG_TO_RAD);
            theta_q[i + 5] = sin(0.60 * DEG_TO_RAD);
        }


        return true;
    }


    void LslidarChDriver::publishLaserScan() {
        if (!is_update_difop || first_frame_flag == 1) {
            return;
        }
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
        if (sweep_data_bac->points.size() <= 1) {
            return;
        }
        scan_msg->header.frame_id = frame_id;

        //scan_msg->header.stamp = pcl_conversions::fromPCL(point_cloud_timestamp);
        scan_msg->header.stamp = ros::Time(point_cloud_timestamp);

        scan_msg->angle_min = DEG2RAD(0);
        scan_msg->angle_max = DEG2RAD(180);
        scan_msg->range_min = min_range;
        scan_msg->range_max = max_range;
        scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
        uint point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        for (size_t j = 0; j < sweep_data_bac->points.size(); ++j) {
            if (sweep_data_bac->points[j].line != channel_num) { continue; }
            float horizontal_angle = sweep_data_bac->points[j].azimuth * DEG_TO_RAD;
            uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
            point_index = (point_index < point_size) ? point_index : (point_index % point_size);
            scan_msg->ranges[point_index] = sweep_data_bac->points[j].distance;
            scan_msg->intensities[point_index] = sweep_data_bac->points[j].intensity;

        }
        laserscan_pub.publish(scan_msg);
    }

    void LslidarChDriver::publishPointCloud() {
        if (!is_update_difop || first_frame_flag == 1) {
            return;
        }
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        if (pcl_type) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;

            pcl::PointXYZI point;
            //   point_cloud->header.stamp = point_cloud_timestamp;
            for (size_t j = 0; j < sweep_data_bac->points.size(); ++j) {
                if ((sweep_data_bac->points[j].azimuth > angle_disable_min) and
                    (sweep_data_bac->points[j].azimuth < angle_disable_max)) {
                    continue;
                }
                point.x = sweep_data_bac->points[j].x;
                point.y = sweep_data_bac->points[j].y;
                point.z = sweep_data_bac->points[j].z;
                point.intensity = sweep_data_bac->points[j].intensity;
                point_cloud->points.push_back(point);
                ++point_cloud->width;
            }

            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            pc_msg.header.stamp = ros::Time(point_cloud_timestamp);
            pointcloud_pub.publish(pc_msg);

        } else {
            pcl::PointCloud<VPoint>::Ptr point_cloud(new pcl::PointCloud<VPoint>);
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;

            //pcl::PointXYZI point;
            VPoint point;
            for (size_t j = 0; j < sweep_data_bac->points.size(); ++j) {
                if ((sweep_data_bac->points[j].azimuth > angle_disable_min) and
                    (sweep_data_bac->points[j].azimuth < angle_disable_max)) {
                    continue;
                }

                point.time = sweep_data_bac->points[j].time;
                point.x = sweep_data_bac->points[j].x;
                point.y = sweep_data_bac->points[j].y;
                point.z = sweep_data_bac->points[j].z;
                point.intensity = sweep_data_bac->points[j].intensity;
                point.ring = sweep_data_bac->points[j].line;
                point_cloud->points.push_back(point);
                ++point_cloud->width;
            }

            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            pc_msg.header.stamp = ros::Time(point_cloud_timestamp);
            pointcloud_pub.publish(pc_msg);

        }
        return;
    }


    int LslidarChDriver::convertCoordinate(struct Firing lidardata) {
        if (!isPointInRange(lidardata.distance) || lidardata.azimuth > 180.0) {
            return -1;
        }
        double x = 0.0, y = 0.0, z = 0.0;
        double z_sin_theat = 0.0;
        double Z_cos_theat = 0.0;
        line_num = lidardata.vertical_line;
        int temp = (line_num / 6) % 2;
        if (temp == 0) {
            z_sin_theat =
                    theta_t[line_num] + 2 * cos((lidardata.azimuth - 0.767) * DEG_TO_RAD * 0.5) * theta_q[line_num];
        } else {
            z_sin_theat =
                    theta_t[line_num] + 2 * cos((lidardata.azimuth + 0.767) * DEG_TO_RAD * 0.5) * theta_q[line_num];
        }
        Z_cos_theat = sqrt(1 - z_sin_theat * z_sin_theat);

        x = lidardata.distance * Z_cos_theat * cos_azimuth_table[static_cast<int>(lidardata.azimuth * 100)];
        y = lidardata.distance * Z_cos_theat * sin_azimuth_table[static_cast<int>(lidardata.azimuth * 100)];
        z = lidardata.distance * z_sin_theat;


        sweep_data->points.push_back(lslidar_msgs::LslidarChPoint());
        lslidar_msgs::LslidarChPoint &new_point =        // new_point 为push_back最后一个的引用
                sweep_data->points[sweep_data->points.size() - 1];

        new_point.x = x;
        new_point.y = y;
        new_point.z = z;
        new_point.azimuth = lidardata.azimuth;
        new_point.distance = lidardata.distance;
        new_point.intensity = lidardata.intensity;
        new_point.line = lidardata.vertical_line;
        new_point.time = lidardata.time;
        return 0;
    }


    bool LslidarChDriver::polling() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_msgs::LslidarChPacketPtr packet(
                new lslidar_msgs::LslidarChPacket());
        int rc = -1;
        // Since the lslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            // keep reading until full packet received

            rc = msop_input_->getPacket(packet);


            if (rc == 0) break;       // got a full packet?
            if (rc < 0) return false; // end of file reached?
        }

        if (use_time_service) {

            //struct tm cur_time{};
            memset(&cur_time, 0, sizeof(cur_time));
            if (packet->data[1205] == 0x01) {
                cur_time.tm_sec = packet->data[1199];
                cur_time.tm_min = packet->data[1198];
                cur_time.tm_hour = packet->data[1197];
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
            } else if (packet->data[1205] == 0x02) {
                cur_time.tm_sec = packet->data[1199];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
            }
            packet_timestamp_s = timegm(&cur_time);
            packet_timestamp_ns = (packet->data[1200] +
                                   packet->data[1201] * pow(2, 8) +
                                   packet->data[1202] * pow(2, 16) +
                                   packet->data[1203] * pow(2, 24)
                                  ) * 1e3;
            packet_timestamp = packet_timestamp_s + packet_timestamp_ns * 1e-9;

            // ch128x1_packet->timestamp = packet_timestamp;
        } else {
            packet_timestamp = ros::Time::now().toSec();
        }

        struct Firing lidardata{};


        packet_interval_time = packet_timestamp - last_packet_timestamp;

       // if(packet_interval_time < 0 || packet_interval_time > 0.01){ROS_INFO("error time =%.f",packet_interval_time);}

        last_packet_timestamp = packet_timestamp;


        // Decode the packet

        if (packet->data[1205] == 0x01) {
            for (size_t point_idx = 0; point_idx < 1197; point_idx += 7) {
                if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) &&
                    (packet->data[point_idx + 2] == 0xbb) && (packet->data[point_idx + 3] == 0x00)) {
                    packetType = true;
                    if (first_frame_flag == 0 || first_frame_flag == 1) {
                        first_frame_flag++;
                    }
                    point_cloud_timestamp =
                            packet_timestamp - packet_interval_time + point_idx * packet_interval_time / 1197.0;
                    // RCLCPP_INFO(get_logger(), "pct=%.6f", point_cloud_timestamp);
                }
                // Compute the time of the point

                point_time = packet_timestamp - packet_interval_time + point_idx * packet_interval_time / 1197.0 -
                             point_cloud_timestamp;


                if (packet->data[point_idx] < 120) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.vertical_line = packet->data[point_idx];
                    lidardata.azimuth = (packet->data[point_idx + 1] * 256 + packet->data[point_idx + 2]) * 0.01f;
                    lidardata.distance =
                            (packet->data[point_idx + 3] * 65536 +
                             packet->data[point_idx + 4] * 256 +
                             packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                    lidardata.intensity = packet->data[point_idx + 6];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
                }
                if (packetType) {
                    {
                        std::unique_lock<std::mutex> lock(pointcloud_lock);
                        sweep_data_bac = sweep_data;
                    }
                    std::thread ppc_thread(&LslidarChDriver::publishPointCloud, this);
                    ppc_thread.detach();
                    //publishPointCloud();

                    if (publish_laserscan) {
                        std::thread pls_thread(&LslidarChDriver::publishLaserScan, this);
                        pls_thread.detach();
                        //publishLaserScan();
                    }
                    packetType = false;
                    sweep_data = lslidar_msgs::LslidarChScanPtr(
                            new lslidar_msgs::LslidarChScan());
                }
            }
        } else if (packet->data[1205] == 0x02) {
            ROS_INFO_ONCE(
                    "lidar is double echo model,and the selected echo is: %d [0 mean double echo; 1 mean first echo; 2 mean second echo]",
                    echo_num);
            for (size_t point_idx = 0; point_idx < 1199; point_idx += 11) {
                if ((packet->data[point_idx] == 0xff) && (packet->data[point_idx + 1] == 0xaa) &&
                    (packet->data[point_idx + 2] == 0xbb) && (packet->data[point_idx + 3] == 0x00)) {
                    if (first_frame_flag == 0 || first_frame_flag == 1) {
                        first_frame_flag++;
                    }
                    packetType = true;
                    point_cloud_timestamp =
                            packet_timestamp - packet_interval_time + point_idx * packet_interval_time / 1199.0;
                }
                // Compute the time of the point

                point_time = packet_timestamp - packet_interval_time +
                             point_idx * packet_interval_time / 1199.0 - point_cloud_timestamp;

                if (packet->data[point_idx] < 120) {
                    if (echo_num == 0) {
                        memset(&lidardata, 0, sizeof(lidardata));
                        lidardata.vertical_line = packet->data[point_idx];
                        lidardata.azimuth =
                                (packet->data[point_idx + 1] * 256 + packet->data[point_idx + 2]) * 0.01f;
                        lidardata.distance =
                                (packet->data[point_idx + 3] * 65536 +
                                 packet->data[point_idx + 4] * 256 +
                                 packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                        lidardata.intensity = packet->data[point_idx + 6];
                        lidardata.time = point_time;
                        convertCoordinate(lidardata);

                        memset(&lidardata, 0, sizeof(lidardata));
                        lidardata.vertical_line = packet->data[point_idx];
                        lidardata.azimuth =
                                (packet->data[point_idx + 1] * 256 + packet->data[point_idx + 2]) * 0.01f;
                        lidardata.distance =
                                (packet->data[point_idx + 7] * 65536 +
                                 packet->data[point_idx + 8] * 256 +
                                 packet->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                        lidardata.intensity = packet->data[point_idx + 10];
                        lidardata.time = point_time;
                        convertCoordinate(lidardata);
                    } else if (echo_num == 1) {
                        memset(&lidardata, 0, sizeof(lidardata));
                        lidardata.vertical_line = packet->data[point_idx];
                        lidardata.azimuth =
                                (packet->data[point_idx + 1] * 256 + packet->data[point_idx + 2]) * 0.01f;
                        lidardata.distance =
                                (packet->data[point_idx + 3] * 65536 +
                                 packet->data[point_idx + 4] * 256 +
                                 packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                        lidardata.intensity = packet->data[point_idx + 6];
                        lidardata.time = point_time;
                        convertCoordinate(lidardata);
                    } else if (echo_num == 2) {
                        memset(&lidardata, 0, sizeof(lidardata));
                        lidardata.vertical_line = packet->data[point_idx];
                        lidardata.azimuth =
                                (packet->data[point_idx + 1] * 256 + packet->data[point_idx + 2]) * 0.01f;
                        lidardata.distance =
                                (packet->data[point_idx + 7] * 65536 +
                                 packet->data[point_idx + 8] * 256 +
                                 packet->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                        lidardata.intensity = packet->data[point_idx + 10];
                        lidardata.time = point_time;
                        convertCoordinate(lidardata);
                    }

                }
                if (packetType) {
                    //("---------------onesweep--------------------------\n");
                    {
                        std::unique_lock<std::mutex> lock(pointcloud_lock);
                        sweep_data_bac = sweep_data;
                    }
                    std::thread ppc_thread(&LslidarChDriver::publishPointCloud, this);
                    ppc_thread.detach();
                    //publishPointCloud();

                    if (publish_laserscan) {
                        std::thread pls_thread(&LslidarChDriver::publishLaserScan, this);
                        pls_thread.detach();
                        //publishLaserScan();
                    }
                    packetType = false;
                    sweep_data = lslidar_msgs::LslidarChScanPtr(
                            new lslidar_msgs::LslidarChScan());
                }
            }
        }

        return true;
    }

    void LslidarChDriver::initTimeStamp(void) {

        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }
    }

    void LslidarChDriver::difopPoll(void) {
        lslidar_msgs::LslidarChPacketPtr difop_packet(
                new lslidar_msgs::LslidarChPacket());

        // reading and publishing scans as fast as possible.
        while (ros::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 0) {
                // getFPGA_GPSTimeStamp(difop_packet);
                if (difop_packet->data[0] == 0xa5 && difop_packet->data[1] == 0xff && difop_packet->data[2] == 0x00 &&
                    difop_packet->data[3] == 0x5a) {
                    this->packetTimeStamp[4] = difop_packet->data[41];
                    this->packetTimeStamp[5] = difop_packet->data[40];
                    this->packetTimeStamp[6] = difop_packet->data[39];
                    this->packetTimeStamp[7] = difop_packet->data[38];
                    this->packetTimeStamp[8] = difop_packet->data[37];
                    this->packetTimeStamp[9] = difop_packet->data[36];
                    is_update_difop = true;
                }

            } else if (rc < 0) {
                return;
            }
        }
    }


} // namespace lslidar_driver
