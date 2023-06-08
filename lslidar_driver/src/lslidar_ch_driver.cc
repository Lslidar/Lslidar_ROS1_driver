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
            sweep_data(new lslidar_msgs::LslidarChRingScan()),
            sweep_data_bac(new lslidar_msgs::LslidarChRingScan()) {

        last_packet_timestamp = 0.0;

        return;
    }

    LslidarChDriver::~LslidarChDriver() {

        if (NULL == difop_thread_) {
            difop_thread_->interrupt();
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
        pnh.param("group_ip", group_ip_string, std::string("234.2.3.2"));
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
        pnh.param("packet_rate", packet_rate, 6737.0);
        pnh.param("use_time_service", use_time_service, false);
        pnh.param<bool>("is_filter", is_filter, false);
        pnh.param<double>("filter_distance", filter_distance, 0.0);
        pnh.param<double>("filter_angle_min", filter_angle_min, 0.0);
        pnh.param<double>("filter_angle_max", filter_angle_max, 0.0);
        pnh.param<double>("filter_intensity", filter_intensity, 0.0);
        pnh.param<int>("filter_line", filter_line, 0);

        ROS_INFO("using time service or not: %d", use_time_service);
        ROS_INFO("lidar type: %s", lidar_type.c_str());
        inet_aton(lidar_ip_string.c_str(), &lidar_ip);
        angle_disable_min = angle_disable_min * DEG_TO_RAD;
        angle_disable_max = angle_disable_max * DEG_TO_RAD;

        if (add_multicast) ROS_INFO_STREAM("Opening UDP socket: group_address " << group_ip_string);


        if (publish_laserscan) {
            if (channel_num < 0) {
                channel_num = 0;
                ROS_WARN("channel_num outside of the index, select channel 0 instead!");
            } else if (channel_num > 63) {
                channel_num = 63;
                ROS_WARN("channel_num outside of the index, select channel 63 instead!");
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

        difop_thread_ = boost::shared_ptr<boost::thread>(
                new boost::thread(boost::bind(&LslidarChDriver::difopPoll, this)));


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


        return true;
    }


    void LslidarChDriver::publishLaserScan() {
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
        if (sweep_data_bac->scans[channel_num].points.size() <= 1) {
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
        for (size_t j = 0; j < sweep_data_bac->scans[channel_num].points.size(); ++j) {

            float horizontal_angle = sweep_data_bac->scans[channel_num].points[j].azimuth;
            uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
            point_index = (point_index < point_size) ? point_index : (point_index % point_size);
            scan_msg->ranges[point_index] = sweep_data_bac->scans[channel_num].points[j].distance;
            scan_msg->intensities[point_index] = sweep_data_bac->scans[channel_num].points[j].intensity;

        }
        laserscan_pub.publish(scan_msg);
    }

    void LslidarChDriver::publishPointCloud() {
        std::unique_lock<std::mutex> lock(pointcloud_lock);
        if (pcl_type) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;

            pcl::PointXYZI point;

            for (int i = 0; i < 64; ++i) {
                if (sweep_data_bac->scans[i].points.size() == 0) { continue; }
                //   point_cloud->header.stamp = point_cloud_timestamp;
                for (size_t j = 0; j < sweep_data_bac->scans[i].points.size(); ++j) {
                    if ((sweep_data_bac->scans[i].points[j].azimuth > angle_disable_min) and
                        (sweep_data_bac->scans[i].points[j].azimuth < angle_disable_max)) {
                        continue;
                    }
                    if (is_filter) {
                        if ((sweep_data_bac->scans[i].points[j].distance < filter_distance) &&
                            (sweep_data_bac->scans[i].points[j].azimuth > filter_angle_min * DEG_TO_RAD) &&
                            (sweep_data_bac->scans[i].points[j].azimuth < filter_angle_max * DEG_TO_RAD) &&
                            (sweep_data_bac->scans[i].points[j].intensity < filter_intensity) &&
                            (sweep_data_bac->scans[i].points[j].line >= filter_line)) {
                            continue;
                        }
                    }
                    point.x = sweep_data_bac->scans[i].points[j].x;
                    point.y = sweep_data_bac->scans[i].points[j].y;
                    point.z = sweep_data_bac->scans[i].points[j].z;
                    point.intensity = sweep_data_bac->scans[i].points[j].intensity;
                    point_cloud->points.push_back(point);
                    ++point_cloud->width;
                }
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
            for (int i = 0; i < 64; ++i) {
                if (sweep_data_bac->scans[i].points.size() == 0) { continue; }
                // point_cloud->header.stamp = point_cloud_timestamp;
                for (size_t j = 0; j < sweep_data_bac->scans[i].points.size(); ++j) {
                    if ((sweep_data_bac->scans[i].points[j].azimuth > angle_disable_min) and
                        (sweep_data_bac->scans[i].points[j].azimuth < angle_disable_max)) {
                        continue;
                    }
                    if (is_filter) {
                        if ((sweep_data_bac->scans[i].points[j].distance < filter_distance) &&
                            (sweep_data_bac->scans[i].points[j].azimuth > filter_angle_min * DEG_TO_RAD) &&
                            (sweep_data_bac->scans[i].points[j].azimuth < filter_angle_max * DEG_TO_RAD) &&
                            (sweep_data_bac->scans[i].points[j].intensity < filter_intensity) &&
                            (sweep_data_bac->scans[i].points[j].line >= filter_line)) {
                            continue;
                        }
                    }
                    point.time = sweep_data_bac->scans[i].points[j].time;
                    point.x = sweep_data_bac->scans[i].points[j].x;
                    point.y = sweep_data_bac->scans[i].points[j].y;
                    point.z = sweep_data_bac->scans[i].points[j].z;
                    point.intensity = sweep_data_bac->scans[i].points[j].intensity;
                    point.ring = sweep_data_bac->scans[i].points[j].line;
                    point_cloud->points.push_back(point);
                    ++point_cloud->width;
                }
            }
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            pc_msg.header.stamp = ros::Time(point_cloud_timestamp);
            pointcloud_pub.publish(pc_msg);

        }
        return;
    }


    int LslidarChDriver::convertCoordinate(struct Firing lidardata) {
        if (!isPointInRange(lidardata.distance) || lidardata.azimuth > 180.0 * DEG_TO_RAD) {
            return -1;
        }
        double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0;
        double z_sin_altitude = 0.0;
        double z_cos_altitude = 0.0;
        if(!isPointInRange(lidardata.distance)){
            return -1;
        }
        line_num = lidardata.vertical_line;
        if (line_num % 8 == 0 || line_num % 8 == 1 || line_num % 8 == 2 || line_num % 8 == 3) {
            z_sin_altitude = sin(-13.33 * DEG_TO_RAD + floor(line_num / 4) * 1.33 * DEG_TO_RAD) +
                             2 * cos(lidardata.azimuth * 0.5 + 1.05 * DEG_TO_RAD) *
                             sin((line_num % 4) * 0.33 * DEG_TO_RAD);

        } else if (line_num % 8 == 4 || line_num % 8 == 5 || line_num % 8 == 6 || line_num % 8 == 7) {
            z_sin_altitude = sin(-13.33 * DEG_TO_RAD + floor(line_num / 4) * 1.33 * DEG_TO_RAD) +
                             2 * cos(lidardata.azimuth * 0.5 - 1.05 * DEG_TO_RAD) *
                             sin((line_num % 4) * 0.33 * DEG_TO_RAD);
        }
        z_cos_altitude = sqrt(1 - z_sin_altitude * z_sin_altitude);
        x_coord = lidardata.distance * z_cos_altitude * cos(lidardata.azimuth);
        y_coord = lidardata.distance * z_cos_altitude * sin(lidardata.azimuth);
        z_coord = lidardata.distance * z_sin_altitude;

        sweep_data->scans[lidardata.vertical_line].points.push_back(lslidar_msgs::LslidarChPoint());
        lslidar_msgs::LslidarChPoint &new_point =        // new_point 为push_back最后一个的引用
                sweep_data->scans[lidardata.vertical_line].points[
                        sweep_data->scans[lidardata.vertical_line].points.size() - 1];
        new_point.x = x_coord;
        new_point.y = y_coord;
        new_point.z = z_coord;
        new_point.intensity = lidardata.intensity;
        new_point.line = lidardata.vertical_line;
        new_point.time = lidardata.time;
        new_point.distance = lidardata.distance;
        new_point.azimuth = lidardata.azimuth;
        return 0;

    }


    bool LslidarChDriver::polling() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_msgs::LslidarChPacketPtr ch64_packet(
                new lslidar_msgs::LslidarChPacket());
        int rc = -1;
        // Since the lslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            // keep reading until full packet received

            rc = msop_input_->getPacket(ch64_packet);


            if (rc == 0) break;       // got a full packet?
            if (rc < 0) return false; // end of file reached?
        }

        if (use_time_service) {

            //struct tm cur_time{};
            memset(&cur_time, 0, sizeof(cur_time));

/*            cur_time.tm_sec = this->packetTimeStamp[4];
            cur_time.tm_min = this->packetTimeStamp[5];
            cur_time.tm_hour = this->packetTimeStamp[6];*/
            cur_time.tm_sec = ch64_packet->data[1199];
            cur_time.tm_min = ch64_packet->data[1198];
            cur_time.tm_hour = ch64_packet->data[1197];
            cur_time.tm_mday = this->packetTimeStamp[7];
            cur_time.tm_mon = this->packetTimeStamp[8] - 1;
            cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;

            packet_timestamp_s = timegm(&cur_time);

            packet_timestamp_ns = (ch64_packet->data[1200] +
                                   ch64_packet->data[1201] * pow(2, 8) +
                                   ch64_packet->data[1202] * pow(2, 16) +
                                   ch64_packet->data[1203] * pow(2, 24)) * 1e3; //ns
            packet_timestamp = packet_timestamp_s + packet_timestamp_ns * 1e-9;
            // ch128x1_packet->timestamp = packet_timestamp;
        } else {
            packet_timestamp = ros::Time::now().toSec();
        }

        struct Firing lidardata{};


        packet_interval_time = packet_timestamp - last_packet_timestamp;
        last_packet_timestamp = packet_timestamp;

        bool packetType = false;

        // Decode the packet
        if (ch64_packet->data[1205] == 0x01) {
            for (size_t point_idx = 0; point_idx < 1197; point_idx += 7) {
                if ((ch64_packet->data[point_idx] == 0xff) && (ch64_packet->data[point_idx + 1] == 0xaa) &&
                    (ch64_packet->data[point_idx + 2] == 0xbb)) {
                    packetType = true;
                    point_cloud_timestamp =
                            packet_timestamp - packet_interval_time + point_idx * packet_interval_time / 1197.0;
                }
                // Compute the time of the point

                if (fabs(last_packet_timestamp) < 1e-6) {
                    point_time = packet_timestamp;
                } else {
                    point_time = packet_timestamp - packet_interval_time +
                                 (point_idx + 7) * packet_interval_time / 1197.0;
                }
                if (ch64_packet->data[point_idx] < 64) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.vertical_line = ch64_packet->data[point_idx];
                    lidardata.azimuth =
                            (ch64_packet->data[point_idx + 1] * 256 + ch64_packet->data[point_idx + 2]) * 0.01 *
                            DEG_TO_RAD;
                    lidardata.distance =
                            (ch64_packet->data[point_idx + 3] * 65536 +
                             ch64_packet->data[point_idx + 4] * 256 +
                             ch64_packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                    lidardata.intensity = ch64_packet->data[point_idx + 6];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
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
                    sweep_data = lslidar_msgs::LslidarChRingScanPtr(
                            new lslidar_msgs::LslidarChRingScan());
                }
            }
        } else if (ch64_packet->data[1205] == 0x02) {
            ROS_INFO_ONCE(
                    "lidar is double echo model,and the selected echo is: %d [0 mean double echo; 1 mean first echo; 2 mean second echo]",
                    echo_num);
            for (size_t point_idx = 0; point_idx < 1199; point_idx += 11) {
                if ((ch64_packet->data[point_idx] == 0xff) && (ch64_packet->data[point_idx + 1] == 0xaa) &&
                    (ch64_packet->data[point_idx + 2] == 0xbb)) {
                    packetType = true;
                    point_cloud_timestamp =
                            packet_timestamp - packet_interval_time + point_idx * packet_interval_time / 1199.0;
                }
                // Compute the time of the point
                if (fabs(last_packet_timestamp) < 1e-6) {
                    point_time = packet_timestamp;
                } else {
                    point_time =
                            packet_timestamp - packet_interval_time + (point_idx + 11) * packet_interval_time / 1199.0;
                }
                if (ch64_packet->data[point_idx] < 64) {
                    if (echo_num == 0) {
                        memset(&lidardata, 0, sizeof(lidardata));
                        lidardata.vertical_line = ch64_packet->data[point_idx];
                        lidardata.azimuth =
                                (ch64_packet->data[point_idx + 1] * 256 + ch64_packet->data[point_idx + 2]) * 0.01 *
                                DEG_TO_RAD;
                        lidardata.distance =
                                (ch64_packet->data[point_idx + 3] * 65536 +
                                 ch64_packet->data[point_idx + 4] * 256 +
                                 ch64_packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                        lidardata.intensity = ch64_packet->data[point_idx + 6];
                        lidardata.time = point_time;
                        convertCoordinate(lidardata);

                        memset(&lidardata, 0, sizeof(lidardata));
                        lidardata.vertical_line = ch64_packet->data[point_idx];
                        lidardata.azimuth =
                                (ch64_packet->data[point_idx + 1] * 256 + ch64_packet->data[point_idx + 2]) * 0.01 *
                                DEG_TO_RAD;
                        lidardata.distance =
                                (ch64_packet->data[point_idx + 7] * 65536 +
                                 ch64_packet->data[point_idx + 8] * 256 +
                                 ch64_packet->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                        lidardata.intensity = ch64_packet->data[point_idx + 10];
                        lidardata.time = point_time;
                        convertCoordinate(lidardata);
                    } else if (echo_num == 1) {
                        memset(&lidardata, 0, sizeof(lidardata));
                        lidardata.vertical_line = ch64_packet->data[point_idx];
                        lidardata.azimuth =
                                (ch64_packet->data[point_idx + 1] * 256 + ch64_packet->data[point_idx + 2]) * 0.01 *
                                DEG_TO_RAD;
                        lidardata.distance =
                                (ch64_packet->data[point_idx + 3] * 65536 +
                                 ch64_packet->data[point_idx + 4] * 256 +
                                 ch64_packet->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                        lidardata.intensity = ch64_packet->data[point_idx + 6];
                        lidardata.time = point_time;
                        convertCoordinate(lidardata);
                    } else if (echo_num == 2) {
                        memset(&lidardata, 0, sizeof(lidardata));
                        lidardata.vertical_line = ch64_packet->data[point_idx];
                        lidardata.azimuth =
                                (ch64_packet->data[point_idx + 1] * 256 + ch64_packet->data[point_idx + 2]) * 0.01 *
                                DEG_TO_RAD;
                        lidardata.distance =
                                (ch64_packet->data[point_idx + 7] * 65536 +
                                 ch64_packet->data[point_idx + 8] * 256 +
                                 ch64_packet->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                        lidardata.intensity = ch64_packet->data[point_idx + 10];
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
                    sweep_data = lslidar_msgs::LslidarChRingScanPtr(
                            new lslidar_msgs::LslidarChRingScan());
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
/*                    this->packetTimeStamp[4] = difop_packet->data[41];
                    this->packetTimeStamp[5] = difop_packet->data[40];
                    this->packetTimeStamp[6] = difop_packet->data[39];*/
                    this->packetTimeStamp[7] = difop_packet->data[38];
                    this->packetTimeStamp[8] = difop_packet->data[37];
                    this->packetTimeStamp[9] = difop_packet->data[36];

                }

            } else if (rc < 0) {
                return;
            }

            ros::spinOnce();
        }
    }


} // namespace lslidar_driver
