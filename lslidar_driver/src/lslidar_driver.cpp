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


#include "lslidar_driver/lslidar_driver.h"
#include <std_msgs/String.h>


namespace lslidar_driver {


    lslidarDriver::lslidarDriver(ros::NodeHandle &node, ros::NodeHandle &private_nh) : nh(node),
                                                                                       pnh(private_nh),
                                                                                       socket_id(-1),
                                                                                       last_azimuth(0),
                                                                                       sweep_end_time(0.0),
                                                                                       is_first_sweep(true),
                                                                                       return_mode(1),
                                                                                       config_vert(true),
                                                                                       sweep_data(
                                                                                               new lslidar_msgs::LslidarScan()) {
        config_vert_num = 0;
        count = 0;
        is_update_gps_time = false;
        last_packet_timestamp = 0.0;
        packet_timestamp = 0.0;
        packet_interval_time = 0.0;

        return;
    }

    bool lslidarDriver::checkPacketValidity(const lslidar_driver::RawPacket *packet) {
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            if (packet->blocks[blk_idx].header != UPPER_BANK) {
                return false;
            }
        }
        return true;
    }

    bool lslidarDriver::isPointInRange(const double &distance) {
        return (distance >= min_range && distance < max_range);
    }


    bool lslidarDriver::loadParameters() {
        pnh.param("pcap", dump_file, std::string(""));
        pnh.param<std::string>("frame_id", frame_id, "laser_link");
        pnh.param<std::string>("lidar_type", lidar_type, "c16");
        pnh.param<std::string>("c16_type", c16_type, "c16_2");
        pnh.param<std::string>("c32_type", c32_type, "c32_2");
        pnh.param<int>("c32_fpga_type", c32_fpga_type, 3);
        pnh.param<bool>("add_multicast", add_multicast, false);
        pnh.param<bool>("config_vert", config_vert, true);
        pnh.param("group_ip", group_ip_string, std::string("234.2.3.2"));
        pnh.param("device_ip", lidar_ip_string, std::string("192.168.1.200"));
        pnh.param("msop_port", msop_udp_port, (int) MSOP_DATA_PORT_NUMBER);
        pnh.param("difop_port", difop_udp_port, (int) DIFOP_DATA_PORT_NUMBER);
        pnh.param("packet_size", packet_size, 1206);
        pnh.param("scan_num", scan_num, 8);
        pnh.param("packet_rate", packet_rate, 840.0);
        pnh.param<double>("horizontal_angle_resolution", horizontal_angle_resolution, 0.20);
        pnh.param("min_range", min_range, 0.3);
        pnh.param("max_range", max_range, 150.0);
        pnh.param("distance_unit", distance_unit, 0.25);
        pnh.param("angle_disable_min", angle_disable_min, 0);
        pnh.param("angle_disable_max", angle_disable_max, 0);
        pnh.param<bool>("use_gps_ts", use_gps_ts, false);
        pnh.param<bool>("pcl_type", pcl_type, false);
        pnh.param<bool>("publish_scan", publish_scan, false);
        pnh.param<bool>("coordinate_opt", coordinate_opt, false);
        pnh.param<std::string>("pointcloud_topic", pointcloud_topic, "lslidar_point_cloud");
        inet_aton(lidar_ip_string.c_str(), &lidar_ip);
        if (add_multicast) ROS_INFO_STREAM("opening UDP socket: group_address " << group_ip_string);

        ROS_INFO("lidar packet size: %d", packet_size);
        ROS_INFO("use gps time or not: %d", use_gps_ts);
        return true;
    }

    void lslidarDriver::initTimeStamp() {
        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }
        this->packet_time_s = 0;
        this->packet_time_ns = 0;
        this->timeStamp = ros::Time(0.0);
        this->timeStamp_bak = ros::Time(0.0);
    }

    bool lslidarDriver::createRosIO() {
        pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 10);
        scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
        lslidar_control = nh.advertiseService("lslidarcontrol", &lslidarC16Driver::lslidarC16Control, this);

        if (dump_file != "") {
            msop_input_.reset(new lslidar_driver::InputPCAP(pnh, msop_udp_port, packet_rate, dump_file));
            difop_input_.reset(new lslidar_driver::InputPCAP(pnh, difop_udp_port, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_driver::InputSocket(pnh, msop_udp_port));
            difop_input_.reset(new lslidar_driver::InputSocket(pnh, difop_udp_port));
        }
        difop_thread_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&lslidarDriver::difopPoll, this)));


        return true;
    }


    void lslidarDriver::publishPointcloud() {
        if (sweep_data->points.size() < 65 || !is_update_gps_time) {
            return;
        }
/*        if (pointcloud_pub.getNumSubscribers() == 0) {
            return;
        }*/

        if (pcl_type) {

            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            // pcl_conversions::toPCL(sweep_data->header).stamp;
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;
            point_cloud->header.stamp = static_cast<uint64_t>(sweep_end_time * 1e6);
            size_t j;
            pcl::PointXYZI point;
            if (sweep_data->points.size() > 0) {
                for (j = 0; j < sweep_data->points.size(); ++j) {
                    if ((sweep_data->points[j].azimuth > angle_disable_min) and
                        (sweep_data->points[j].azimuth < angle_disable_max)) {
                        continue;
                    }
                    point.x = sweep_data->points[j].x;
                    point.y = sweep_data->points[j].y;
                    point.z = sweep_data->points[j].z;
                    point.intensity = sweep_data->points[j].intensity;
                    point_cloud->points.push_back(point);
                    ++point_cloud->width;
                }
            }
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            pc_msg.header.stamp = ros::Time(sweep_end_time);
            pointcloud_pub.publish(pc_msg);
        } else {
            VPointcloud::Ptr point_cloud(new VPointcloud());
            //pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            // pcl_conversions::toPCL(sweep_data->header).stamp;
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;
            point_cloud->header.stamp = static_cast<uint64_t>(sweep_end_time * 1e6);
            size_t j;
            VPoint point;
            if (sweep_data->points.size() > 0) {
                for (j = 0; j < sweep_data->points.size(); ++j) {
                    if ((sweep_data->points[j].azimuth > angle_disable_min) and
                        (sweep_data->points[j].azimuth < angle_disable_max)) {
                        continue;
                    }
                    point.x = sweep_data->points[j].x;
                    point.y = sweep_data->points[j].y;
                    point.z = sweep_data->points[j].z;
                    point.intensity = sweep_data->points[j].intensity;
                    point.ring = sweep_data->points[j].ring;
                    point.time = sweep_data->points[j].time;
                    point_cloud->points.push_back(point);
                    ++point_cloud->width;
                }
            }
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            pc_msg.header.stamp = ros::Time(sweep_end_time);
            pointcloud_pub.publish(pc_msg);

        }
        return;
    }


    void lslidarDriver::publishScan() {
        if (sweep_data->points.size() < 65) {
            return;
        }
        sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
        scan_msg->header.frame_id = frame_id;
        int layer_num_local = scan_num;
        ROS_INFO_ONCE("default channel is %d", layer_num_local);

        scan_msg->header.stamp = ros::Time(sweep_end_time);
        scan_msg->angle_min = 0.0;
        scan_msg->angle_max = 2.0 * M_PI;
        scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
        scan_msg->range_min = min_range;
        scan_msg->range_max = max_range;

        uint point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        if (coordinate_opt) {
            for (size_t j = 0; j < sweep_data->points.size(); ++j) {
                if (layer_num_local == sweep_data->points[j].ring) {
                    float horizontal_angle = sweep_data->points[j].azimuth * 0.01 * DEG_TO_RAD;
                    uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_index = (point_index <= point_size) ? point_index : (point_index % point_size);
                    scan_msg->ranges[point_size - point_index - 1] = sweep_data->points[j].distance;
                    scan_msg->intensities[point_size - point_index - 1] = sweep_data->points[j].intensity;
                }
            }
        } else {
            for (size_t j = 0; j < sweep_data->points.size(); ++j) {
                if (layer_num_local == sweep_data->points[j].ring) {
/*                    float h_angle =
                            (sweep_data->points[j].azimuth + 9000.0) < 36000.0 ? sweep_data->points[j].azimuth + 9000.0
                                                                               : sweep_data->points[j].azimuth - 27000.0;
                    if(h_angle < 18000.0){
                        h_angle = 18000.0 - h_angle;
                    }else{
                        h_angle = 36000.0 - (h_angle - 18000.0);
                    }*/
                    float h_angle = (45000.0 - sweep_data->points[j].azimuth) < 36000.0 ? 45000.0 -
                                                                                          sweep_data->points[j].azimuth
                                                                                        :
                                    9000.0 - sweep_data->points[j].azimuth;
                    // ROS_INFO("a=%f,b=%f",sweep_data->points[j].azimuth,h_angle);
                    float horizontal_angle = h_angle * 0.01 * DEG_TO_RAD;
                    uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_index = (point_index <= point_size) ? point_index : (point_index % point_size);
                    scan_msg->ranges[point_index] = sweep_data->points[j].distance;
                    scan_msg->intensities[point_index] = sweep_data->points[j].intensity;
                }

            }

        }

        scan_pub.publish(scan_msg);

    }

    bool lslidarDriver::lslidarC16Control(lslidar_msgs::lslidar_control::Request &req,
                                          lslidar_msgs::lslidar_control::Response &res) {
        ROS_WARN("--------------------------");
        // sleep(1);
        lslidar_msgs::LslidarPacketPtr packet0(new lslidar_msgs::LslidarPacket);
        packet0->data[0] = 0x00;
        packet0->data[1] = 0x00;
        int rc_msop = -1;

        if (req.LaserControl == 1) {
            if ((rc_msop = msop_input_->getPacket(packet0)) == 0) {
                res.status = 1;
                ROS_WARN("receive cmd: %d,already power on status", req.LaserControl);
                return true;
            }
            ROS_WARN("receive cmd: %d,power on", req.LaserControl);
            SendPacketTolidar(true);
            double time1 = ros::Time::now().toSec();
            do {
                rc_msop = msop_input_->getPacket(packet0);
                double time2 = ros::Time::now().toSec();
                if (time2 - time1 > 20) {
                    res.status = 0;
                    ROS_WARN("lidar connect error");
                    return true;
                }
            } while ((rc_msop != 0) && (packet0->data[0] != 0xff) && (packet0->data[1] != 0xee));
            sleep(5);
            res.status = 1;
        } else if (req.LaserControl == 0) {
            ROS_WARN("receive cmd: %d,power off", req.LaserControl);
            SendPacketTolidar(false);
            res.status = 1;
        } else {
            ROS_WARN("cmd error");
            res.status = 0;
        }
        return true;


    }

    bool lslidarDriver::SendPacketTolidar(bool power_switch) {
        int socketid;
        unsigned char config_data[packet_size];
        //int data_port = difop_data[24] * 256 + difop_data[25];
        mempcpy(config_data, difop_data, packet_size);
        config_data[0] = 0xAA;
        config_data[1] = 0x00;
        config_data[2] = 0xFF;
        config_data[3] = 0x11;
        config_data[4] = 0x22;
        config_data[5] = 0x22;
        config_data[6] = 0xAA;
        config_data[7] = 0xAA;
        config_data[8] = 0x02;
        config_data[9] = 0x58;
        if (power_switch) {
            config_data[45] = 0x00;
        } else {
            config_data[45] = 0x01;
        }

        sockaddr_in addrSrv;
        socketid = socket(2, 2, 0);
        addrSrv.sin_addr.s_addr = inet_addr(lidar_ip_string.c_str());
        addrSrv.sin_family = AF_INET;
        addrSrv.sin_port = htons(2368);
        sendto(socketid, (const char *) config_data, 1206, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
        return false;

    }


    lslidarC16Driver::lslidarC16Driver(ros::NodeHandle &node, ros::NodeHandle &private_nh) : lslidarDriver(node,
                                                                                                           private_nh) {
        return;
    }


    lslidarC16Driver::~lslidarC16Driver() {
        if (difop_thread_ != nullptr) {
            difop_thread_->join();
        }
    }


    bool lslidarC16Driver::initialize() {

        this->initTimeStamp();
        if (!loadParameters()) {
            ROS_ERROR("cannot load all required ROS parameters.");
            return false;
        }
        if (c16_type == "c16_2") {
            for (int j = 0; j < 16; ++j) {
                c16_vertical_angle[j] = c16_2_vertical_angle[j];
                c16_config_vertical_angle[j] = c16_2_vertical_angle[j];
                c16_sin_scan_altitude[j] = sin(c16_vertical_angle[j] * DEG_TO_RAD);
                c16_cos_scan_altitude[j] = cos(c16_vertical_angle[j] * DEG_TO_RAD);
            }
        } else {
            for (int j = 0; j < 16; ++j) {
                c16_vertical_angle[j] = c16_1_vertical_angle[j];
                c16_config_vertical_angle[j] = c16_1_vertical_angle[j];
                c16_sin_scan_altitude[j] = sin(c16_vertical_angle[j] * DEG_TO_RAD);
                c16_cos_scan_altitude[j] = cos(c16_vertical_angle[j] * DEG_TO_RAD);
            }
        }


        if (!createRosIO()) {
            ROS_ERROR("cannot create all ROS IO.");
            return false;
        }

        // create the sin and cos table for different azimuth values
        for (int j = 0; j < 36000; ++j) {
            sin_azimuth_table[j] = sin(j * 0.01 * DEG_TO_RAD);
            cos_azimuth_table[j] = cos(j * 0.01 * DEG_TO_RAD);
        }
        return true;
    }


/*    bool lslidarC16Driver::checkPacketValidity(const RawPacket *packet) {
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            if (packet->blocks[blk_idx].header != UPPER_BANK) {
                return false;
            }
        }
        return true;
    }*/


    void lslidarC16Driver::difopPoll() {
        // reading and publishing scans as fast as possible.
        lslidar_msgs::LslidarPacketPtr difop_packet_ptr(new lslidar_msgs::LslidarPacket);
        while (ros::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet_ptr);
            if (rc == 0) {
                if (difop_packet_ptr->data[0] != 0xa5 || difop_packet_ptr->data[1] != 0xff ||
                    difop_packet_ptr->data[2] != 0x00 || difop_packet_ptr->data[3] != 0x5a) {
                    return;
                }
                count++;
                if (count == 2) { is_update_gps_time = true; }
                for (int i = 0; i < 1206; i++) {
                    difop_data[i] = difop_packet_ptr->data[i];
                }

                int version_data = difop_packet_ptr->data[1202];
                if (config_vert) {
                    if (2 == version_data) {
                        for (int j = 0; j < 16; ++j) {
                            uint8_t data1 = difop_packet_ptr->data[234 + 2 * j];
                            uint8_t data2 = difop_packet_ptr->data[234 + 2 * j + 1];
                            int vert_angle = data1 * 256 + data2;
                            vert_angle = vert_angle > 32767 ? (vert_angle - 65535) : vert_angle;
                            c16_config_vertical_angle[j] = (double) vert_angle * 0.01f;
                            if (fabs(c16_vertical_angle[j] - c16_config_vertical_angle[j]) > 1.5) {
                                //c16_config_vertical_angle[j] = c16_vertical_angle[j];
                                config_vert_num++;
                            }

                        }
                        if (config_vert_num == 0) {
                            for (int k = 0; k < 16; ++k) {
                                c16_sin_scan_altitude[k] = sin(c16_config_vertical_angle[k] * DEG_TO_RAD);
                                c16_cos_scan_altitude[k] = cos(c16_config_vertical_angle[k] * DEG_TO_RAD);
                            }
                        }

                    } else {
                        for (int j = 0; j < 16; ++j) {
                            uint8_t data1 = difop_packet_ptr->data[245 + 2 * j];
                            uint8_t data2 = difop_packet_ptr->data[245 + 2 * j + 1];
                            int vert_angle = data1 * 256 + data2;
                            vert_angle = vert_angle > 32767 ? (vert_angle - 65535) : vert_angle;
                            c16_config_vertical_angle[j] = (double) vert_angle * 0.01f;
                            if (fabs(c16_vertical_angle[j] - c16_config_vertical_angle[j]) > 1.5) {
                                config_vert_num++;
                            }
                        }
                        if (config_vert_num == 0) {
                            for (int k = 0; k < 16; ++k) {
                                c16_sin_scan_altitude[k] = sin(c16_config_vertical_angle[k] * DEG_TO_RAD);
                                c16_cos_scan_altitude[k] = cos(c16_config_vertical_angle[k] * DEG_TO_RAD);
                            }
                        }
                    }
                    config_vert = false;
                }
                if (packet_size == 1206) {
                    if (2 == version_data) {
                        this->packetTimeStamp[4] = difop_packet_ptr->data[41];
                        this->packetTimeStamp[5] = difop_packet_ptr->data[40];
                        this->packetTimeStamp[6] = difop_packet_ptr->data[39];
                        this->packetTimeStamp[7] = difop_packet_ptr->data[38];
                        this->packetTimeStamp[8] = difop_packet_ptr->data[37];
                        this->packetTimeStamp[9] = difop_packet_ptr->data[36];
                    } else {
                        this->packetTimeStamp[4] = difop_packet_ptr->data[57];
                        this->packetTimeStamp[5] = difop_packet_ptr->data[56];
                        this->packetTimeStamp[6] = difop_packet_ptr->data[55];
                        this->packetTimeStamp[7] = difop_packet_ptr->data[54];
                        this->packetTimeStamp[8] = difop_packet_ptr->data[53];
                        this->packetTimeStamp[9] = difop_packet_ptr->data[52];
                    }

                }
            } else if (rc < 0) {
                return;
            }
        }
    }


    void lslidarC16Driver::decodePacket(const RawPacket *packet) {
        //couputer azimuth angle for each firing
        //even numbers
        for (size_t b_idx = 0; b_idx < BLOCKS_PER_PACKET; ++b_idx) {
            firings.firing_azimuth[b_idx] = packet->blocks[b_idx].rotation % 36000; //* 0.01 * DEG_TO_RAD;
        }

        // computer distance ,intensity
        //12 blocks
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            const RawBlock &raw_block = packet->blocks[blk_idx];

            int32_t azimuth_diff = 0;
            if (1 == return_mode) {
                if (blk_idx < BLOCKS_PER_PACKET - 1) {
                    azimuth_diff = firings.firing_azimuth[blk_idx + 1] - firings.firing_azimuth[blk_idx];
                    azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
                } else {
                    azimuth_diff = firings.firing_azimuth[blk_idx] - firings.firing_azimuth[blk_idx - 1];
                    azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
                }
            } else {
                if (blk_idx < BLOCKS_PER_PACKET - 2) {
                    azimuth_diff = firings.firing_azimuth[blk_idx + 2] - firings.firing_azimuth[blk_idx];
                    azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
                } else {
                    azimuth_diff = firings.firing_azimuth[blk_idx] - firings.firing_azimuth[blk_idx - 2];
                    azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
                }

            }
            for (size_t scan_idx = 0; scan_idx < SCANS_PER_BLOCK; ++scan_idx) {
                size_t byte_idx = RAW_SCAN_SIZE * scan_idx;
                //azimuth
                firings.azimuth[blk_idx * 32 + scan_idx] =
                        firings.firing_azimuth[blk_idx] + scan_idx * azimuth_diff / FIRING_TOFFSET;
                firings.azimuth[blk_idx * 32 + scan_idx] = firings.azimuth[blk_idx * 32 + scan_idx] % 36000;
                // distance
/*                TwoBytes raw_distance{0};
                raw_distance.bytes[0] = raw_block.data[byte_idx];
                raw_distance.bytes[1] = raw_block.data[byte_idx + 1];*/
                firings.distance[blk_idx * 32 + scan_idx] =
                        static_cast<double>(raw_block.data[byte_idx] + raw_block.data[byte_idx + 1] * 256) *
                        DISTANCE_RESOLUTION * distance_unit;

                //intensity
                firings.intensity[blk_idx * 32 + scan_idx] = static_cast<double>(
                        raw_block.data[byte_idx + 2]);
            }

        }
        return;
    }

/** poll the device
 *  @returns true unless end of file reached
 */
    bool lslidarC16Driver::poll(void) {  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_msgs::LslidarPacketPtr packet(new lslidar_msgs::LslidarPacket());

        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) break;
            if (rc < 0) {
                return false;
            }
        }

        const RawPacket *raw_packet = (const RawPacket *) (&(packet->data[0]));

        //check if the packet is valid
        if (!checkPacketValidity(raw_packet)) {
            return false;
        }

        // packet timestamp
        if (use_gps_ts) {
            lslidar_msgs::LslidarPacket pkt = *packet;
            memset(&cur_time, 0, sizeof(cur_time));
            if (packet_size == 1206) {
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_sec = this->packetTimeStamp[4];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (pkt.data[1200] +
                                  pkt.data[1201] * pow(2, 8) +
                                  pkt.data[1202] * pow(2, 16) +
                                  pkt.data[1203] * pow(2, 24)) * 1e3; //ns
                timeStamp = ros::Time(packet_time_s, packet_time_ns);
                //使用gps授时
                if ((timeStamp - timeStamp_bak).toSec() < 0.0 && (timeStamp - timeStamp_bak).toSec() > -1.0 &&
                    packet_time_ns < 100000000 && is_update_gps_time) {
                    timeStamp = ros::Time(packet_time_s + 1, packet_time_ns);
                } else if ((timeStamp - timeStamp_bak).toSec() > 1.0 && (timeStamp - timeStamp_bak).toSec() < 1.2
                           && packet_time_ns > 900000000 && is_update_gps_time) {
                    timeStamp = ros::Time(packet_time_s - 1, packet_time_ns);
                }
                timeStamp_bak = timeStamp;
                packet->stamp = timeStamp;
                packet_timestamp = packet->stamp.toSec();
            } else if (packet_size == 1212) {
                cur_time.tm_year = pkt.data[1200] + 2000 - 1900;
                cur_time.tm_mon = pkt.data[1201] - 1;
                cur_time.tm_mday = pkt.data[1202];
                cur_time.tm_hour = pkt.data[1203];
                cur_time.tm_min = pkt.data[1204];
                cur_time.tm_sec = pkt.data[1205];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (pkt.data[1206] +
                                  pkt.data[1207] * pow(2, 8) +
                                  pkt.data[1208] * pow(2, 16) +
                                  pkt.data[1209] * pow(2, 24)) * 1e3; //ns
                packet_timestamp = packet_time_s + packet_time_ns * 1e-9;
                timeStamp = ros::Time(packet_time_s, packet_time_ns);
                packet->stamp = timeStamp;
            }
        } else {
            packet->stamp = ros::Time::now();
            packet_timestamp = packet->stamp.toSec();
        }

        packet_interval_time = (packet_timestamp - last_packet_timestamp) / 384.0;
        last_packet_timestamp = packet_timestamp;

        if (packet_size == 1206) {
            if (packet->data[1204] == 0x39) {
                return_mode = 2;
            }
        } else if (packet_size == 1212) {
            if (packet->data[1210] == 0x39) {
                return_mode = 2;
            }
        }

        ROS_INFO_ONCE("return mode: %d", return_mode);


        //decode the packet
        decodePacket(raw_packet);
        // find the start of a new revolution
        // if there is one, new_sweep_start will be the index of the start firing,
        // otherwise, new_sweep_start will be FIRINGS_PER_PACKET.
        size_t new_sweep_start = 0;
        do {
            if (last_azimuth - firings.azimuth[new_sweep_start] > 35000) {
                break;
            } else {
                last_azimuth = firings.azimuth[new_sweep_start];
                ++new_sweep_start;
            }
        } while (new_sweep_start < SCANS_PER_PACKET);

        // The first sweep may not be complete. So, the firings with
        // the first sweep will be discarded. We will wait for the
        // second sweep in order to find the 0 azimuth angle.
        size_t start_fir_idx = 0;
        size_t end_fir_idx = new_sweep_start;
        if (is_first_sweep && new_sweep_start == SCANS_PER_PACKET) {
            return true;
        } else {
            if (is_first_sweep) {
                is_first_sweep = false;
                start_fir_idx = new_sweep_start;
                end_fir_idx = SCANS_PER_PACKET;
                //sweep_start_time = packet->stamp.toSec() - (end_fir_idx - start_fir_idx) * 3.125 * 1e-6;
            }
        }
        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            //check if the point is valid
            if (!isPointInRange(firings.distance[fir_idx]))continue;
            //convert the point to xyz coordinate
            size_t table_idx = firings.azimuth[fir_idx];
            double cos_azimuth = cos_azimuth_table[table_idx];
            double sin_azimuth = sin_azimuth_table[table_idx];


            double x_coord, y_coord, z_coord;
            if (coordinate_opt) {
                int tmp_idx = 1468 - firings.azimuth[fir_idx] < 0 ? 1468 - firings.azimuth[fir_idx] + 36000 : 1468 - firings.azimuth[fir_idx];
                x_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * cos_azimuth +
                          R1_ * cos_azimuth_table[tmp_idx];
                y_coord = -firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * sin_azimuth +
                          R1_ * sin_azimuth_table[tmp_idx];
                z_coord = firings.distance[fir_idx] * c16_sin_scan_altitude[fir_idx % 16];

            } else {
                //Y-axis correspondence 0 degree
                int tmp_idx = firings.azimuth[fir_idx] - 1468 < 0 ? firings.azimuth[fir_idx] -1468 + 36000 : firings.azimuth[fir_idx] -1468;
                x_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * sin_azimuth +
                          R1_ * sin_azimuth_table[tmp_idx];
                y_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * cos_azimuth +
                          R1_ * cos_azimuth_table[tmp_idx];
                z_coord = firings.distance[fir_idx] * c16_sin_scan_altitude[fir_idx % 16];
            }
            // computer the time of the point
            double time = packet_timestamp  -
                          (SCANS_PER_PACKET - fir_idx - 1) * packet_interval_time - sweep_end_time;

            int remapped_scan_idx = (fir_idx % 16) % 2 == 0 ? (fir_idx % 16) / 2 : (fir_idx % 16) / 2 + 8;

            sweep_data->points.push_back(lslidar_msgs::LslidarPoint());
            lslidar_msgs::LslidarPoint &new_point = sweep_data->points[
                    sweep_data->points.size() - 1];
            //pack the data into point msg
            new_point.time = time;
            new_point.x = x_coord;
            new_point.y = y_coord;
            new_point.z = z_coord;
            new_point.intensity = firings.intensity[fir_idx];
            new_point.ring = remapped_scan_idx;
            new_point.azimuth = firings.azimuth[fir_idx];
            new_point.distance = firings.distance[fir_idx];
        }
        // a new sweep begins ----------------------------------------------------

        if (end_fir_idx != SCANS_PER_PACKET) {
            //publish Last frame scan
            sweep_end_time =  packet_timestamp - (SCANS_PER_PACKET - end_fir_idx -1) * packet_interval_time;

            sweep_end_time = sweep_end_time > 0 ? sweep_end_time : 0;
            publishPointcloud();
            if (publish_scan) publishScan();

            sweep_data = lslidar_msgs::LslidarScanPtr(new lslidar_msgs::LslidarScan());
            //prepare the next frame scan
            //sweep_start_time = packet->stamp.toSec() - (end_fir_idx - start_fir_idx) * 3.125 * 1e-6;
            last_azimuth = firings.azimuth[SCANS_PER_PACKET - 1];
            start_fir_idx = end_fir_idx;
            end_fir_idx = SCANS_PER_PACKET;
            for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {

                //check if the point is valid
                if (!isPointInRange(firings.distance[fir_idx]))continue;
                //convert the point to xyz coordinate
                size_t table_idx = firings.azimuth[fir_idx];
                double cos_azimuth = cos_azimuth_table[table_idx];
                double sin_azimuth = sin_azimuth_table[table_idx];
                double x_coord, y_coord, z_coord;
                if (coordinate_opt) {
                    int tmp_idx = 1468 - firings.azimuth[fir_idx] < 0 ? 1468 - firings.azimuth[fir_idx] + 36000 : 1468 - firings.azimuth[fir_idx];
                    x_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * cos_azimuth +
                              R1_ * cos_azimuth_table[tmp_idx];
                    y_coord = -firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * sin_azimuth +
                              R1_ * sin_azimuth_table[tmp_idx];
                    z_coord = firings.distance[fir_idx] * c16_sin_scan_altitude[fir_idx % 16];

                } else {
                    //Y-axis correspondence 0 degree
                    int tmp_idx = firings.azimuth[fir_idx] - 1468 < 0 ? firings.azimuth[fir_idx] -1468 + 36000 : firings.azimuth[fir_idx] -1468;
                    x_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * sin_azimuth +
                              R1_ * sin_azimuth_table[tmp_idx];
                    y_coord = firings.distance[fir_idx] * c16_cos_scan_altitude[fir_idx % 16] * cos_azimuth +
                              R1_ * cos_azimuth_table[tmp_idx];
                    z_coord = firings.distance[fir_idx] * c16_sin_scan_altitude[fir_idx % 16];
                }
                // computer the time of the point
                double time = packet_timestamp  -
                              (SCANS_PER_PACKET - fir_idx -1) * packet_interval_time - sweep_end_time;
                int remapped_scan_idx = (fir_idx % 16) % 2 == 0 ? (fir_idx % 16) / 2 : (fir_idx % 16) / 2 + 8;
                sweep_data->points.push_back(lslidar_msgs::LslidarPoint());
                lslidar_msgs::LslidarPoint &new_point = sweep_data->points[
                        sweep_data->points.size() - 1];
                //pack the data into point msg
                new_point.time = time;
                new_point.x = x_coord;
                new_point.y = y_coord;
                new_point.z = z_coord;
                new_point.intensity = firings.intensity[fir_idx];
                new_point.ring = remapped_scan_idx;
                new_point.azimuth = firings.azimuth[fir_idx];
                new_point.distance = firings.distance[fir_idx];
            }

        }
        //packet_pub.publish(*packet);
        return true;
    }


    lslidarC32Driver::lslidarC32Driver(ros::NodeHandle &node, ros::NodeHandle &private_nh) : lslidarDriver(node,
                                                                                                           private_nh) {
        return;
    }

    lslidarC32Driver::~lslidarC32Driver() {
        if (difop_thread_ != nullptr) {
            difop_thread_->join();
        }
    }

    bool lslidarC32Driver::initialize() {
        this->initTimeStamp();
        if (!loadParameters()) {
            ROS_ERROR("cannot load all required ROS parameters.");
            return false;
        }
        if (c32_type == "c32_2") {
            if (c32_fpga_type == 2) {
                for (int j = 0; j < 32; ++j) {
                    c32_vertical_angle[j] = c32_2_vertical_angle_26[j];
                    c32_config_vertical_angle[j] = c32_2_vertical_angle_26[j];
                    c32_sin_scan_altitude[j] = sin(c32_vertical_angle[j] * DEG_TO_RAD);
                    c32_cos_scan_altitude[j] = cos(c32_vertical_angle[j] * DEG_TO_RAD);
                }
            } else {
                for (int j = 0; j < 32; ++j) {
                    c32_vertical_angle[j] = c32_2_vertical_angle[j];
                    c32_config_vertical_angle[j] = c32_2_vertical_angle[j];
                    c32_sin_scan_altitude[j] = sin(c32_vertical_angle[j] * DEG_TO_RAD);
                    c32_cos_scan_altitude[j] = cos(c32_vertical_angle[j] * DEG_TO_RAD);
                }
            }
        } else {
            if (c32_fpga_type == 2) {
                for (int j = 0; j < 32; ++j) {
                    c32_vertical_angle[j] = c32_1_vertical_angle_26[j];
                    c32_config_vertical_angle[j] = c32_1_vertical_angle_26[j];
                    c32_sin_scan_altitude[j] = sin(c32_vertical_angle[j] * DEG_TO_RAD);
                    c32_cos_scan_altitude[j] = cos(c32_vertical_angle[j] * DEG_TO_RAD);
                }
            } else {
                for (int j = 0; j < 32; ++j) {
                    c32_vertical_angle[j] = c32_1_vertical_angle[j];
                    c32_config_vertical_angle[j] = c32_1_vertical_angle[j];
                    c32_sin_scan_altitude[j] = sin(c32_vertical_angle[j] * DEG_TO_RAD);
                    c32_cos_scan_altitude[j] = cos(c32_vertical_angle[j] * DEG_TO_RAD);
                }
            }
        }

        if (!createRosIO()) {
            ROS_ERROR("cannot create all ROS IO.");
            return false;
        }

        for (int i = 0; i < 4; ++i) {
            adjust_angle[i] = 0.0;
        }

        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) / 100.0 * DEG_TO_RAD;
            sin_azimuth_table[j] = sin(angle);
            cos_azimuth_table[j] = cos(angle);
        }

        return true;
    }

    void lslidarC32Driver::difopPoll() {
        // reading and publishing scans as fast as possible.
        lslidar_msgs::LslidarPacketPtr difop_packet_ptr(new lslidar_msgs::LslidarPacket);
        while (ros::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet_ptr);
            if (rc == 0) {
                if (difop_packet_ptr->data[0] != 0xa5 || difop_packet_ptr->data[1] != 0xff ||
                    difop_packet_ptr->data[2] != 0x00 || difop_packet_ptr->data[3] != 0x5a) {
                    return;
                }
                count++;
                if (count == 2) { is_update_gps_time = true; }
                for (int i = 0; i < 1206; i++) {
                    difop_data[i] = difop_packet_ptr->data[i];
                }
                ROS_INFO_ONCE("c32 vertical angle resolution type: %s; c32 fpga type: %0.1f", c32_type.c_str(),
                              difop_data[1202] + int(difop_data[1203] / 16) * 0.1);
                //int version_data = difop_packet_ptr->data[1202];
                if (config_vert) {
                    if (3 == c32_fpga_type) {
                        for (int i = 0; i < 32; ++i) {
                            uint8_t data1 = difop_packet_ptr->data[245 + 2 * i];
                            uint8_t data2 = difop_packet_ptr->data[245 + 2 * i + 1];
                            int vert_angle = data1 * 256 + data2;
                            vert_angle = vert_angle > 32767 ? (vert_angle - 65535) : vert_angle;
                            c32_config_tmp_angle[i] = (double) vert_angle * 0.01f;

                        }
                        for (int j = 0; j < 32; ++j) {
                            c32_config_vertical_angle[j] = c32_config_tmp_angle[adjust_angle_index[j]];

                            if (fabs(c32_vertical_angle[j] - c32_config_vertical_angle[j]) > 3) {
                                config_vert_num++;
                            }
                        }
                        if (config_vert_num == 0) {
                            for (int k = 0; k < 32; ++k) {
                                c32_sin_scan_altitude[k] = sin(c32_config_vertical_angle[k] * DEG_TO_RAD);
                                c32_cos_scan_altitude[k] = cos(c32_config_vertical_angle[k] * DEG_TO_RAD);
                            }
                        }
                        // horizontal correction angle
                        int angle_a0 = difop_packet_ptr->data[186] * 256 + difop_packet_ptr->data[187];
                        adjust_angle[0] = angle_a0 > 32767 ? 32767 - angle_a0 : angle_a0;

                        int angle_a1 = difop_packet_ptr->data[190] * 256 + difop_packet_ptr->data[191];
                        adjust_angle[1] = angle_a1 > 32767 ? 32767 - angle_a1 : angle_a1;

                        int angle_a2 = difop_packet_ptr->data[188] * 256 + difop_packet_ptr->data[189];
                        adjust_angle[2] = angle_a2 > 32767 ? 32767 - angle_a2 : angle_a2;

                        int angle_a3 = difop_packet_ptr->data[192] * 256 + difop_packet_ptr->data[193];
                        adjust_angle[3] = angle_a3 > 32767 ? 32767 - angle_a3 : angle_a3;
                    } else {
                        for (int j = 0; j < 32; ++j) {
                            uint8_t data1 = difop_packet_ptr->data[882 + 2 * j];
                            uint8_t data2 = difop_packet_ptr->data[882 + 2 * j + 1];
                            int vert_angle = data1 * 256 + data2;
                            vert_angle = vert_angle > 32767 ? (vert_angle - 65535) : vert_angle;
                            c32_config_vertical_angle[j] = (double) vert_angle * 0.01f;

                            if (fabs(c32_vertical_angle[j] - c32_config_vertical_angle[j]) > 3.0) {
                                config_vert_num++;
                            }
                        }
                        if (config_vert_num == 0) {
                            for (int k = 0; k < 32; ++k) {
                                c32_sin_scan_altitude[k] = sin(c32_config_vertical_angle[k] * DEG_TO_RAD);
                                c32_cos_scan_altitude[k] = cos(c32_config_vertical_angle[k] * DEG_TO_RAD);

                            }
                        }
                        // horizontal correction angle
                        int angle_a0 = difop_packet_ptr->data[34] * 256 + difop_packet_ptr->data[35];
                        adjust_angle[0] = angle_a0 > 32767 ? 32767 - angle_a0 : angle_a0;

                        int angle_a1 = difop_packet_ptr->data[42] * 256 + difop_packet_ptr->data[43];
                        adjust_angle[1] = angle_a1 > 32767 ? 32767 - angle_a1 : angle_a1;

                        int angle_a2 = difop_packet_ptr->data[66] * 256 + difop_packet_ptr->data[67];
                        adjust_angle[2] = angle_a2 > 32767 ? 32767 - angle_a2 : angle_a2;

                        int angle_a3 = difop_packet_ptr->data[68] * 256 + difop_packet_ptr->data[69];
                        adjust_angle[3] = angle_a3 > 32767 ? 32767 - angle_a3 : angle_a3;
                    }
                    config_vert = false;
                }
                if (packet_size == 1206) {
                    if (2 == c32_fpga_type) {
                        this->packetTimeStamp[4] = difop_packet_ptr->data[41];
                        this->packetTimeStamp[5] = difop_packet_ptr->data[40];
                        this->packetTimeStamp[6] = difop_packet_ptr->data[39];
                        this->packetTimeStamp[7] = difop_packet_ptr->data[38];
                        this->packetTimeStamp[8] = difop_packet_ptr->data[37];
                        this->packetTimeStamp[9] = difop_packet_ptr->data[36];
                    } else {
                        this->packetTimeStamp[4] = difop_packet_ptr->data[57];
                        this->packetTimeStamp[5] = difop_packet_ptr->data[56];
                        this->packetTimeStamp[6] = difop_packet_ptr->data[55];
                        this->packetTimeStamp[7] = difop_packet_ptr->data[54];
                        this->packetTimeStamp[8] = difop_packet_ptr->data[53];
                        this->packetTimeStamp[9] = difop_packet_ptr->data[52];
                    }
                }

            } else if (rc < 0) {
                return;
            }
        }
    }


    void lslidarC32Driver::decodePacket(const RawPacket *packet) {
        //couputer azimuth angle for each firing, 12

        for (size_t fir_idx = 0; fir_idx < BLOCKS_PER_PACKET; ++fir_idx) {
            firings.firing_azimuth[fir_idx] = packet->blocks[fir_idx].rotation % 36000; //* 0.01 * DEG_TO_RAD;
            // ROS_INFO("azi==%d",packet->blocks[fir_idx].rotation);
        }
        // ROS_WARN("-----------------");
        // computer distance ,intensity
        //12 blocks
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            const RawBlock &raw_block = packet->blocks[blk_idx];

            int32_t azimuth_diff = 0;
            if (1 == return_mode) {
                if (blk_idx < BLOCKS_PER_PACKET - 1) {
                    azimuth_diff = firings.firing_azimuth[blk_idx + 1] - firings.firing_azimuth[blk_idx];
                } else {
                    azimuth_diff = firings.firing_azimuth[blk_idx] - firings.firing_azimuth[blk_idx - 1];
                }
                azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
            } else {
                if (blk_idx < BLOCKS_PER_PACKET - 2) {
                    azimuth_diff = firings.firing_azimuth[blk_idx + 2] - firings.firing_azimuth[blk_idx];

                } else {
                    azimuth_diff = firings.firing_azimuth[blk_idx] - firings.firing_azimuth[blk_idx - 2];
                }
                azimuth_diff = azimuth_diff < 0 ? azimuth_diff + 36000 : azimuth_diff;
            }
            // 32 scan
            for (size_t scan_idx = 0; scan_idx < SCANS_PER_BLOCK; ++scan_idx) {
                size_t byte_idx = RAW_SCAN_SIZE * scan_idx;
                //azimuth
                firings.azimuth[blk_idx * 32 + scan_idx] = firings.firing_azimuth[blk_idx] +
                                                           scan_idx * azimuth_diff / FIRING_TOFFSET;

                firings.azimuth[blk_idx * 32 + scan_idx] = firings.azimuth[blk_idx * 32 + scan_idx] % 36000;

                /*
                // calibration azimuth ，1°
              if ("c32_2" == c32_type) {
                    // -----结果是否是正数 ？
                    int adjust_diff = adjust_angle[1] - adjust_angle[0];
                    if (adjust_diff > 300 && adjust_diff < 460) {
                        // fpga :v 2.7
                        if (3 == c32_fpga_type) {
                            if ( 1 >= scan_fir_idx % 4 ) {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                            } else {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                            }
                           // ROS_INFO("id: %d--azi: %d",blk_idx * 32 + scan_fir_idx,firings.azimuth[blk_idx * 32 + scan_fir_idx]);
                        } else {
                            if (0 == scan_fir_idx % 2) {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                            } else {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                            }
                        }
                    } else {
                        // fpga: v2.6
                        if (0 == scan_fir_idx % 2) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        } else {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[0];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }
                    }
                }
                //  calibration azimuth ,0.33°
                if ("c32_1" == c32_type) {
                    int adjust_diff_one = adjust_angle[1] - adjust_angle[0];
                    int adjust_diff_two = adjust_angle[3] - adjust_angle[2];
                    if (3 == c32_fpga_type) {
                        // fpga v3.0
                        if (0 == scan_fir_idx || 1 == scan_fir_idx || 4 == scan_fir_idx || 8 == scan_fir_idx ||
                            9 == scan_fir_idx || 12 == scan_fir_idx
                            || 16 == scan_fir_idx || 17 == scan_fir_idx || 21 == scan_fir_idx || 24 == scan_fir_idx ||
                            25 == scan_fir_idx || 29 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[3];
                        }
                        if (2 == scan_fir_idx || 3 == scan_fir_idx || 6 == scan_fir_idx || 10 == scan_fir_idx ||
                            11 == scan_fir_idx || 14 == scan_fir_idx
                            || 18 == scan_fir_idx || 19 == scan_fir_idx || 23 == scan_fir_idx || 26 == scan_fir_idx ||
                            27 == scan_fir_idx || 31 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[2];
                        }
                        if (5 == scan_fir_idx || 13 == scan_fir_idx || 20 == scan_fir_idx || 28 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                        }
                        if (7 == scan_fir_idx || 15 == scan_fir_idx || 22 == scan_fir_idx || 30 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        }
                    } else if (adjust_diff_one > 500 && adjust_diff_one < 660 && adjust_diff_two > 150 &&
                               adjust_diff_two < 350) {
                        //fpga v2.7
                        if (10 == scan_fir_idx || 14 == scan_fir_idx || 18 == scan_fir_idx || 22 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[3];
                        }
                        if (11 == scan_fir_idx || 15 == scan_fir_idx || 19 == scan_fir_idx || 23 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[2];
                        }
                        if (0 == scan_fir_idx || 2 == scan_fir_idx || 4 == scan_fir_idx || 6 == scan_fir_idx ||
                            8 == scan_fir_idx || 12 == scan_fir_idx
                            || 16 == scan_fir_idx || 20 == scan_fir_idx || 24 == scan_fir_idx || 26 == scan_fir_idx ||
                            28 == scan_fir_idx || 30 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                        }
                        if (1 == scan_fir_idx || 3 == scan_fir_idx || 5 == scan_fir_idx || 7 == scan_fir_idx ||
                            9 == scan_fir_idx || 13 == scan_fir_idx
                            || 17 == scan_fir_idx || 21 == scan_fir_idx || 25 == scan_fir_idx || 27 == scan_fir_idx ||
                            29 == scan_fir_idx || 31 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        }
                    } else {
                        // fpga v2.6
                        if (10 == scan_fir_idx || 14 == scan_fir_idx || 18 == scan_fir_idx || 22 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        }
                        if (11 == scan_fir_idx || 15 == scan_fir_idx || 19 == scan_fir_idx || 23 == scan_fir_idx) {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[0];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }
                        if (0 == scan_fir_idx || 2 == scan_fir_idx || 4 == scan_fir_idx || 6 == scan_fir_idx ||
                            8 == scan_fir_idx || 12 == scan_fir_idx
                            || 16 == scan_fir_idx || 20 == scan_fir_idx || 24 == scan_fir_idx || 26 == scan_fir_idx ||
                            28 == scan_fir_idx || 30 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                        }
                        if (1 == scan_fir_idx || 3 == scan_fir_idx || 5 == scan_fir_idx || 7 == scan_fir_idx ||
                            9 == scan_fir_idx || 13 == scan_fir_idx
                            || 17 == scan_fir_idx || 21 == scan_fir_idx || 25 == scan_fir_idx || 27 == scan_fir_idx ||
                            29 == scan_fir_idx || 31 == scan_fir_idx) {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[1];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }

                    }

                }


                firings.azimuth[blk_idx * 32 + scan_fir_idx] = firings.azimuth[blk_idx * 32 + scan_fir_idx] % 36000;
                 */

                // distance
                firings.distance[blk_idx * 32 + scan_idx] =
                        static_cast<double>(raw_block.data[byte_idx] + raw_block.data[byte_idx + 1] * 256) *
                        DISTANCE_RESOLUTION * distance_unit;

                //intensity
                firings.intensity[blk_idx * 32 + scan_idx] = static_cast<double>(
                        raw_block.data[byte_idx + 2]);
            }

        }
        return;
    }

/** poll the device
 *  @returns true unless end of file reached
 */
    bool lslidarC32Driver::poll(void) {
        lslidar_msgs::LslidarPacketPtr packet(new lslidar_msgs::LslidarPacket());


        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) break;
            if (rc < 0) return false;
        }
        const RawPacket *raw_packet = (const RawPacket *) (&(packet->data[0]));

        //check if the packet is valid
        if (!checkPacketValidity(raw_packet)) return false;

        // packet timestamp
        if (use_gps_ts) {
            lslidar_msgs::LslidarPacket pkt = *packet;
            memset(&cur_time, 0, sizeof(cur_time));
            if (packet_size == 1206) {
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_sec = this->packetTimeStamp[4];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (pkt.data[1200] +
                                  pkt.data[1201] * pow(2, 8) +
                                  pkt.data[1202] * pow(2, 16) +
                                  pkt.data[1203] * pow(2, 24)) * 1e3; //ns
                timeStamp = ros::Time(packet_time_s, packet_time_ns);
                //使用gps授时
                if ((timeStamp - timeStamp_bak).toSec() < 0.0 && (timeStamp - timeStamp_bak).toSec() > -1.0 &&
                    packet_time_ns < 100000000 && is_update_gps_time) {
                    timeStamp = ros::Time(packet_time_s + 1, packet_time_ns);
                } else if ((timeStamp - timeStamp_bak).toSec() > 1.0 && (timeStamp - timeStamp_bak).toSec() < 1.2
                           && packet_time_ns > 900000000 && is_update_gps_time) {
                    timeStamp = ros::Time(packet_time_s - 1, packet_time_ns);
                }
                timeStamp_bak = timeStamp;
                packet->stamp = timeStamp;
                packet_timestamp = packet->stamp.toSec();
            } else if (packet_size == 1212) {
                cur_time.tm_year = pkt.data[1200] + 2000 - 1900;
                cur_time.tm_mon = pkt.data[1201] - 1;
                cur_time.tm_mday = pkt.data[1202];
                cur_time.tm_hour = pkt.data[1203];
                cur_time.tm_min = pkt.data[1204];
                cur_time.tm_sec = pkt.data[1205];
                packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
                packet_time_ns = (pkt.data[1206] +
                                  pkt.data[1207] * pow(2, 8) +
                                  pkt.data[1208] * pow(2, 16) +
                                  pkt.data[1209] * pow(2, 24)) * 1e3; //ns
                packet_timestamp = packet_time_s + packet_time_ns * 1e-9;
                timeStamp = ros::Time(packet_time_s, packet_time_ns);
                packet->stamp = timeStamp;
            }
        } else {
            packet->stamp = ros::Time::now();
            packet_timestamp = packet->stamp.toSec();
        }

        packet_interval_time = (packet_timestamp - last_packet_timestamp) / 384.0;
        last_packet_timestamp = packet_timestamp;

        if (packet_size == 1206) {
            if (packet->data[1204] == 0x39) {
                return_mode = 2;
            }
        } else if (packet_size == 1212) {
            if (packet->data[1210] == 0x39) {
                return_mode = 2;
            }
        }
        ROS_INFO_ONCE("return mode: %d", return_mode);


        //decode the packet
        decodePacket(raw_packet);
        // find the start of a new revolution
        // if there is one, new_sweep_start will be the index of the start firing,
        // otherwise, new_sweep_start will be FIRINGS_PER_PACKET.
        size_t new_sweep_start = 0;
        do {
            if (last_azimuth - firings.azimuth[new_sweep_start] > 35000) {
                break;
            } else {
                last_azimuth = firings.azimuth[new_sweep_start];
                ++new_sweep_start;
            }
        } while (new_sweep_start < SCANS_PER_PACKET);

        // The first sweep may not be complete. So, the firings with
        // the first sweep will be discarded. We will wait for the
        // second sweep in order to find the 0 azimuth angle.
        size_t start_fir_idx = 0;
        size_t end_fir_idx = new_sweep_start;
        if (is_first_sweep && new_sweep_start == SCANS_PER_PACKET) {
            return true;
        } else {
            if (is_first_sweep) {
                is_first_sweep = false;
                start_fir_idx = new_sweep_start;
                end_fir_idx = SCANS_PER_PACKET;
                //sweep_start_time = packet->stamp.toSec() - (end_fir_idx - start_fir_idx) * 3.125 * 1e-6;
            }
        }
        last_azimuth_tmp = firings.azimuth[SCANS_PER_PACKET - 1];

        for (int blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            for (int scan_fir_idx = 0; scan_fir_idx < SCANS_PER_BLOCK; ++scan_fir_idx) {

                // calibration azimuth ，1°
                if ("c32_2" == c32_type) {

//                    int adjust_diff = adjust_angle[1] - adjust_angle[0];

//                    if (adjust_diff > 300 && adjust_diff < 460) {
                        // fpga :v 2.8 3.0
                        if (3 == c32_fpga_type) {
                            if (1 >= scan_fir_idx % 4) {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                            } else {
                                firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                            }
                            // ROS_INFO("id: %d--azi: %d",blk_idx * 32 + scan_fir_idx,firings.azimuth[blk_idx * 32 + scan_fir_idx]);
                        }
//                    }
                else {
                        // fpga: v2.6
                        if (0 == scan_fir_idx % 2) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        } else {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[0];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }
                    }
                }
                //  calibration azimuth ,0.33°
                if ("c32_1" == c32_type) {
                    int adjust_diff_one = adjust_angle[1] - adjust_angle[0];
                    int adjust_diff_two = adjust_angle[3] - adjust_angle[2];
                    if (3 == c32_fpga_type) {
                        // fpga v 2.8 3.0
                        if (0 == scan_fir_idx || 1 == scan_fir_idx || 4 == scan_fir_idx || 8 == scan_fir_idx ||
                            9 == scan_fir_idx || 12 == scan_fir_idx
                            || 16 == scan_fir_idx || 17 == scan_fir_idx || 21 == scan_fir_idx || 24 == scan_fir_idx ||
                            25 == scan_fir_idx || 29 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[3];
                        }
                        if (2 == scan_fir_idx || 3 == scan_fir_idx || 6 == scan_fir_idx || 10 == scan_fir_idx ||
                            11 == scan_fir_idx || 14 == scan_fir_idx
                            || 18 == scan_fir_idx || 19 == scan_fir_idx || 23 == scan_fir_idx || 26 == scan_fir_idx ||
                            27 == scan_fir_idx || 31 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[2];
                        }
                        if (5 == scan_fir_idx || 13 == scan_fir_idx || 20 == scan_fir_idx || 28 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                        }
                        if (7 == scan_fir_idx || 15 == scan_fir_idx || 22 == scan_fir_idx || 30 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        }
                    } else {
                        // fpga v2.6
                        if (10 == scan_fir_idx || 14 == scan_fir_idx || 18 == scan_fir_idx || 22 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[0];
                        }
                        if (11 == scan_fir_idx || 15 == scan_fir_idx || 19 == scan_fir_idx || 23 == scan_fir_idx) {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[0];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }
                        if (0 == scan_fir_idx || 2 == scan_fir_idx || 4 == scan_fir_idx || 6 == scan_fir_idx ||
                            8 == scan_fir_idx || 12 == scan_fir_idx
                            || 16 == scan_fir_idx || 20 == scan_fir_idx || 24 == scan_fir_idx || 26 == scan_fir_idx ||
                            28 == scan_fir_idx || 30 == scan_fir_idx) {
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] += adjust_angle[1];
                        }
                        if (1 == scan_fir_idx || 3 == scan_fir_idx || 5 == scan_fir_idx || 7 == scan_fir_idx ||
                            9 == scan_fir_idx || 13 == scan_fir_idx
                            || 17 == scan_fir_idx || 21 == scan_fir_idx || 25 == scan_fir_idx || 27 == scan_fir_idx ||
                            29 == scan_fir_idx || 31 == scan_fir_idx) {
                            int tmp_azimuth = firings.azimuth[blk_idx * 32 + scan_fir_idx] - adjust_angle[1];
                            firings.azimuth[blk_idx * 32 + scan_fir_idx] =
                                    tmp_azimuth < 0 ? tmp_azimuth + 36000 : tmp_azimuth;
                        }

                    }

                }
                if(firings.azimuth[blk_idx * 32 + scan_fir_idx] < 0)	    firings.azimuth[blk_idx * 32 + scan_fir_idx]+=36000;
                if(firings.azimuth[blk_idx * 32 + scan_fir_idx] > 36000)	firings.azimuth[blk_idx * 32 + scan_fir_idx]-=36000;
            }

        }


        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            //check if the point is valid
            if (!isPointInRange(firings.distance[fir_idx]))continue;
            if (firings.azimuth[fir_idx] >= 36000) {
                firings.azimuth[fir_idx] = firings.azimuth[fir_idx] - 36000;
            }
            //convert the point to xyz coordinate
            size_t table_idx = firings.azimuth[fir_idx];
            double cos_azimuth = cos_azimuth_table[table_idx];
            double sin_azimuth = sin_azimuth_table[table_idx];
            double x_coord, y_coord, z_coord;
            if (coordinate_opt) {
            int tmp_idx = 1298 - firings.azimuth[fir_idx] < 0 ? 1298 - firings.azimuth[fir_idx] + 36000 : 1298 - firings.azimuth[fir_idx];
                x_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * cos_azimuth +
                          R2_ * cos((12.98 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                y_coord = -firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * sin_azimuth +
                          R2_ * sin((12.98 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                z_coord = firings.distance[fir_idx] * c32_sin_scan_altitude[fir_idx % 32];

            } else {
                //Y-axis correspondence 0 degree
               int tmp_idx = firings.azimuth[fir_idx] - 1298 < 0 ? firings.azimuth[fir_idx] -1298 + 36000 : firings.azimuth[fir_idx] -1298;
                x_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * sin_azimuth +
                          R2_ * sin((firings.azimuth[fir_idx] * 0.01 - 12.98) * DEG_TO_RAD);
                y_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * cos_azimuth +
                          R2_ * cos((firings.azimuth[fir_idx] * 0.01 - 12.98) * DEG_TO_RAD);
                z_coord = firings.distance[fir_idx] * c32_sin_scan_altitude[fir_idx % 32];
            }
            // computer the time of the point
            double time = packet_timestamp  -
                          (SCANS_PER_PACKET - fir_idx - 1) * packet_interval_time - sweep_end_time;


            int remapped_scan_idx;
            if (c32_fpga_type == 2) {
                remapped_scan_idx = fir_idx % 32;
            } else {
                remapped_scan_idx = (fir_idx % 32) % 2 == 0 ? (fir_idx % 32) / 2 : (fir_idx % 32) / 2 + 16;
            }
            sweep_data->points.push_back(lslidar_msgs::LslidarPoint());
            lslidar_msgs::LslidarPoint &new_point = sweep_data->points[
                    sweep_data->points.size() - 1];
            //pack the data into point msg
            new_point.time = time;
            new_point.x = x_coord;
            new_point.y = y_coord;
            new_point.z = z_coord;
            new_point.intensity = firings.intensity[fir_idx];
            new_point.ring = remapped_scan_idx;
            new_point.azimuth = firings.azimuth[fir_idx];
            new_point.distance = firings.distance[fir_idx];
        }
        // a new sweep begins ----------------------------------------------------

        if (end_fir_idx != SCANS_PER_PACKET) {
            //publish Last frame scan

            sweep_end_time =  packet_timestamp - (SCANS_PER_PACKET - end_fir_idx -1) * packet_interval_time;
            sweep_end_time = sweep_end_time > 0 ? sweep_end_time : 0;
            publishPointcloud();
            if (publish_scan) publishScan();

            sweep_data = lslidar_msgs::LslidarScanPtr(new lslidar_msgs::LslidarScan());
            //prepare the next frame scan
            //sweep_start_time = packet->stamp.toSec() - (end_fir_idx - start_fir_idx) * 3.125 * 1e-6;
            last_azimuth = last_azimuth_tmp;
            start_fir_idx = end_fir_idx;
            end_fir_idx = SCANS_PER_PACKET;
            for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {

                //check if the point is valid
                if (!isPointInRange(firings.distance[fir_idx]))continue;
                if (firings.azimuth[fir_idx] >= 36000) {
                    firings.azimuth[fir_idx] = firings.azimuth[fir_idx] - 36000;
                }
                //convert the point to xyz coordinate
                size_t table_idx = firings.azimuth[fir_idx];
                double cos_azimuth = cos_azimuth_table[table_idx];
                double sin_azimuth = sin_azimuth_table[table_idx];
                double x_coord, y_coord, z_coord;
                if (coordinate_opt) {
                  int tmp_idx = 1298 - firings.azimuth[fir_idx] < 0 ? 1298 - firings.azimuth[fir_idx] + 36000 : 1298 - firings.azimuth[fir_idx];
                    x_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * cos_azimuth +
                              R2_ * cos((12.98 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                    y_coord = -firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * sin_azimuth +
                              R2_ * sin((12.98 - firings.azimuth[fir_idx] * 0.01) * DEG_TO_RAD);
                    z_coord = firings.distance[fir_idx] * c32_sin_scan_altitude[fir_idx % 32];

                } else {
                    //Y-axis correspondence 0 degree
                    int tmp_idx = firings.azimuth[fir_idx] - 1298 < 0 ? firings.azimuth[fir_idx] -1298 + 36000 : firings.azimuth[fir_idx] -1298;
                    x_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * sin_azimuth +
                              R2_ * sin((firings.azimuth[fir_idx] * 0.01 - 12.98) * DEG_TO_RAD);
                    y_coord = firings.distance[fir_idx] * c32_cos_scan_altitude[fir_idx % 32] * cos_azimuth +
                              R2_ * cos((firings.azimuth[fir_idx] * 0.01 - 12.98) * DEG_TO_RAD);
                    z_coord = firings.distance[fir_idx] * c32_sin_scan_altitude[fir_idx % 32];

                }
                // computer the time of the point
                double time = packet_timestamp  -
                              (SCANS_PER_PACKET - fir_idx - 1) * packet_interval_time - sweep_end_time;
                int remapped_scan_idx;
                if (c32_fpga_type == 2) {
                    remapped_scan_idx = fir_idx % 32;
                } else {
                    remapped_scan_idx = (fir_idx % 32) % 2 == 0 ? (fir_idx % 32) / 2 : (fir_idx % 32) / 2 + 16;
                }
                sweep_data->points.push_back(lslidar_msgs::LslidarPoint());
                lslidar_msgs::LslidarPoint &new_point = sweep_data->points[
                        sweep_data->points.size() - 1];
                //pack the data into point msg
                new_point.time = time;
                new_point.x = x_coord;
                new_point.y = y_coord;
                new_point.z = z_coord;
                new_point.intensity = firings.intensity[fir_idx];
                new_point.ring = remapped_scan_idx;
                new_point.azimuth = firings.azimuth[fir_idx];
                new_point.distance = firings.distance[fir_idx];
            }

        }
        //packet_pub.publish(*packet);
        return true;
    }


}  // namespace lslidar_driver
