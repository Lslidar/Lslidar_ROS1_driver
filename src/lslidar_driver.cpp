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

namespace lslidar_driver {
    LslidarDriver::LslidarDriver(ros::NodeHandle &node, ros::NodeHandle &private_nh) : nh(node),
                                                                                       pnh(private_nh),
                                                                                       SCANS_PER_PACKET(360),
                                                                                       last_azimuth(0),
                                                                                       sweep_end_time(0.0),
                                                                                       is_first_sweep(true),
                                                                                       packet_rate(60.0),
                                                                                       current_packet_time(0.0),
                                                                                       last_packet_time(0.0),
                                                                                       horizontal_angle_resolution(0.0),
                                                                                       is_get_difop_(false),
                                                                                       threadPool_(std::make_unique<ThreadPool>(2)),   
                                                                                       scan_msg(new sensor_msgs::LaserScan),
                                                                                       scan_msg_bak(new sensor_msgs::LaserScan){
        ROS_INFO("*********** N301 ROS driver version: %s ***********", lslidar_n301_driver_VERSION);
    }

    bool LslidarDriver::loadParameters() {
        pnh.param<std::string>("pcap", dump_file, std::string(""));
        pnh.param<double>("packet_rate", packet_rate, 840.0);
        pnh.param<std::string>("frame_id", frame_id, "laser_link");
        pnh.param<std::string>("laserscan_topic", laserscan_topic, std::string("scan"));
        pnh.param<bool>("add_multicast", add_multicast, false);
        pnh.param<std::string>("group_ip", group_ip_string, std::string("224.0.0.1"));
        pnh.param<std::string>("device_ip", lidar_ip_string, std::string("192.168.1.200"));
        pnh.param<int>("msop_port", msop_udp_port, (int) MSOP_DATA_PORT_NUMBER);
        pnh.param<int>("difop_port", difop_udp_port, (int) DIFOP_DATA_PORT_NUMBER);
        pnh.param<int>("protocol", protocol, 1);
        pnh.param<int>("point_num", point_num, 2000);
        pnh.param<double>("distance_min", min_range, 0.15);
        pnh.param<double>("distance_max", max_range, 200.0);
        pnh.param<int>("angle_disable_min", angle_disable_min, 0);
        pnh.param<int>("angle_disable_max", angle_disable_max, 0);
        pnh.param<double>("horizontal_angle_resolution", horizontal_angle_resolution, 0.18);
        pnh.param<bool>("use_time_service", use_time_service, false);
        pnh.param<bool>("coordinate_opt", coordinate_opt, false);
        inet_aton(lidar_ip_string.c_str(), &lidar_ip);
        
        ROS_INFO_STREAM("Only accepting packets from IP address: " << lidar_ip_string.c_str());
        if (add_multicast) ROS_INFO_STREAM("opening UDP socket: group_address " << group_ip_string);

        return true;
    }

    void LslidarDriver::initTimeStamp() {
        for (int i = 0; i < 10; i++) {
            this->packetTimeStamp[i] = 0;
        }
        this->packet_time_s = 0;
        this->packet_time_ns = 0;
        this->timeStamp = ros::Time(0.0);
    }

    bool LslidarDriver::createRosIO() {
        scan_pub = nh.advertise<sensor_msgs::LaserScan>(laserscan_topic, 10);

        if (!dump_file.empty()) {
            msop_input_.reset(new lslidar_driver::InputPCAP(pnh, msop_udp_port, 1206, packet_rate, dump_file));
            difop_input_.reset(new lslidar_driver::InputPCAP(pnh, difop_udp_port, 1206, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_driver::InputSocket(pnh, msop_udp_port, 1206));
            difop_input_.reset(new lslidar_driver::InputSocket(pnh, difop_udp_port, 1206));
        }

        threadPool_->enqueue([this]() { difopPoll(); });

        return true;
    }

    bool LslidarDriver::initialize() {
        this->initTimeStamp();

        if (!loadParameters()) {
            ROS_WARN("cannot load all required ROS parameters.");
            return false;
        }

        if (!createRosIO()) {
            ROS_WARN("cannot create all ROS IO.");
            return false;
        }

        if (protocol == 1) {   // 1.7
            year  = 1149;
            month = 1150;
            day   = 1151;
            SCANS_PER_PACKET = 360;
            decodePacket = std::bind(&LslidarDriver::decodePacket_1_7, this, std::placeholders::_1);
            ROS_INFO("Parse using protocol 1.7");
        } else if (protocol == 2) {    // 1.6
            year  = 1194;
            month = 1195;
            day   = 1196;
            SCANS_PER_PACKET = 24;
            decodePacket = std::bind(&LslidarDriver::decodePacket_1_6, this, std::placeholders::_1);
            ROS_INFO("Parse using protocol 1.6");
        } else {
            ROS_WARN("Invalid protocol!!!");
            return false;
        }

        while (angle_disable_min < 0)
			angle_disable_min += 36000;
		while (angle_disable_max < 0)
			angle_disable_max += 36000;
		while (angle_disable_min > 36000)
			angle_disable_min -= 36000;
		while (angle_disable_max > 36000)
			angle_disable_max -= 36000;
		if (angle_disable_max == angle_disable_min)
		{
			angle_able_min = 0;
			angle_able_max = 36000;
		}
		else
		{
			if (angle_disable_min < angle_disable_max && angle_disable_min != 0)
			{
				angle_able_min = angle_disable_max;
				angle_able_max = angle_disable_min + 36000;
			}
			if (angle_disable_min < angle_disable_max && angle_disable_min == 0)
			{
				angle_able_min = angle_disable_max;
				angle_able_max = 36000;
			}
			if (angle_disable_min > angle_disable_max)
			{
				angle_able_min = angle_disable_max;
				angle_able_max = angle_disable_min;
			}
		}

        scan_msg->angle_min = 0;
        scan_msg->angle_max = M_PI * 2;
        scan_msg->range_min = min_range;
        scan_msg->range_max = max_range;
        scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
        
        point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        
        return true;
    }

    void LslidarDriver::difopPoll() {
        // reading and publishing scans as fast as possible.
        lslidar_n301_driver::LslidarPacketPtr difop_packet_ptr(new lslidar_n301_driver::LslidarPacket);

        while (ros::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet_ptr);
            if (rc == 0) {
                if (difop_packet_ptr->data[0] != 0xa5 || difop_packet_ptr->data[1] != 0xff ||
                    difop_packet_ptr->data[2] != 0x00 || difop_packet_ptr->data[3] != 0x5a) {
                    return;
                }
                
                // 暂时用不到数据包内容
                for (int i = 0; i < 1206; i++) {
                    difop_data[i] = difop_packet_ptr->data[i];
                }

                is_get_difop_ = true;
            } else if (rc < 0) {
                return;
            }
        }
    }

    // 发布laserscan
    void LslidarDriver::publishScan() {
        std::unique_lock<std::mutex> lock(laserscan_lock);
        scan_msg_bak->header.frame_id = frame_id;
        scan_msg_bak->header.stamp = ros::Time(sweep_end_time);
        scan_pub.publish(scan_msg_bak);
    }

    // 判断是否为雷达数据包
    bool LslidarDriver::checkPacketValidity(lslidar_n301_driver::LslidarPacketPtr &packet) {
        for (size_t blk_idx = 0; blk_idx < BLOCKS_PER_PACKET; ++blk_idx) {
            if (packet->data[blk_idx * SIZE_BLOCK] != 0xff && packet->data[blk_idx * SIZE_BLOCK + 1] != 0xee) {
                return false;
            }
        }
        return true;
    }

    // 计算每包360个点的角度 距离 强度  1.7协议 每个块中30个点
    void LslidarDriver::decodePacket_1_7(lslidar_n301_driver::LslidarPacketPtr &packet) {
        //couputer azimuth angle for each firing
        for (size_t b_idx = 0; b_idx < BLOCKS_PER_PACKET; ++b_idx) {
            firings.firing_azimuth[b_idx] = (packet->data[b_idx * SIZE_BLOCK + 2] + (packet->data[b_idx * SIZE_BLOCK + 3] << 8)) % 36000;
        }
        
        for (size_t block_idx = 0; block_idx < BLOCKS_PER_PACKET; ++block_idx) {
            int32_t azimuth_diff_b = 0;
            if (block_idx < BLOCKS_PER_PACKET - 1) {
                azimuth_diff_b = firings.firing_azimuth[block_idx + 1] - firings.firing_azimuth[block_idx];
            } else {
                azimuth_diff_b = firings.firing_azimuth[block_idx] - firings.firing_azimuth[block_idx - 1];
            }
            azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000 : azimuth_diff_b;

            // 每一块30个点
            for (size_t scan_fir_idx = 0; scan_fir_idx < SCANS_PER_FIRING_1_7; ++scan_fir_idx) {
                if (scan_fir_idx == 15) continue;   // 跳过非点信息 每块第 49 50 51 byte为 年 月 日
                size_t byte_idx = RAW_SCAN_SIZE * scan_fir_idx;
                size_t idx = scan_fir_idx;
                if (scan_fir_idx > 15) idx -= 1;
                firings.azimuth[block_idx * FIRING_TOFFSET_1_7 + idx] = firings.firing_azimuth[block_idx] + idx * azimuth_diff_b / FIRING_TOFFSET_1_7;
                                                                    
                firings.azimuth[block_idx * FIRING_TOFFSET_1_7 + idx] = firings.azimuth[block_idx * FIRING_TOFFSET_1_7 + idx] % 36000;
                // distance
                firings.distance[block_idx * FIRING_TOFFSET_1_7 + idx] = static_cast<float >((packet->data[block_idx * SIZE_BLOCK + byte_idx + 4] + 
                                                                        (packet->data[block_idx * SIZE_BLOCK + byte_idx + 5] << 8)) * DISTANCE_RESOLUTION_1_7);
                                            
                //intensity
                firings.intensity[block_idx * FIRING_TOFFSET_1_7 + idx] = static_cast<float>(packet->data[block_idx * SIZE_BLOCK + byte_idx + 6]); 
            }
        }
    }

    // 计算每包24个点的角度 距离 强度  1.6协议 每个块中2个点
    void LslidarDriver::decodePacket_1_6(lslidar_n301_driver::LslidarPacketPtr &packet) {
        // 数据块的方位角
        for (size_t b_idx = 0; b_idx < BLOCKS_PER_PACKET; ++b_idx) {
            firings.firing_azimuth[b_idx] = packet->data[b_idx * SIZE_BLOCK + 2] + (packet->data[b_idx * SIZE_BLOCK + 3] << 8);
        }

        // 每个数据块中的点数据
        for (size_t block_idx = 0; block_idx < BLOCKS_PER_PACKET; ++block_idx) {
            int32_t azimuth_diff_b = 0; // 相邻两个方位角的差值
            if (block_idx < BLOCKS_PER_PACKET - 1) {
                azimuth_diff_b = firings.firing_azimuth[block_idx + 1] - firings.firing_azimuth[block_idx];
            } else {
                azimuth_diff_b = firings.firing_azimuth[block_idx] - firings.firing_azimuth[block_idx - 1];
            }
            azimuth_diff_b = azimuth_diff_b < 0 ? azimuth_diff_b + 36000 : azimuth_diff_b;

            size_t point_idx = block_idx * 2;

            // 第一个点
            firings.azimuth[point_idx] = firings.firing_azimuth[block_idx] % 36000;
            firings.distance[point_idx] = static_cast<float>((packet->data[block_idx * SIZE_BLOCK + 4]  + 
                                                             (packet->data[block_idx * SIZE_BLOCK + 5] << 8))  * DISTANCE_RESOLUTION_1_6);
            firings.intensity[point_idx] = static_cast<float>(packet->data[block_idx * SIZE_BLOCK + 6]);

            // 第二个点
            firings.azimuth[point_idx + 1] = firings.firing_azimuth[block_idx] + azimuth_diff_b * 0.5;
            firings.azimuth[point_idx + 1] = firings.azimuth[point_idx + 1] % 36000;
            firings.distance[point_idx + 1] = static_cast<float>((packet->data[block_idx * SIZE_BLOCK + 52] + 
                                                                 (packet->data[block_idx * SIZE_BLOCK + 53] << 8)) * DISTANCE_RESOLUTION_1_6);
            firings.intensity[point_idx + 1] = static_cast<float>(packet->data[block_idx * SIZE_BLOCK + 54]);
        }
    }

    bool LslidarDriver::poll() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_n301_driver::LslidarPacketPtr packet(new lslidar_n301_driver::LslidarPacket());
        // Since the rslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) {
                break;
            } else {
                return false;
            }
        }
        //check if the packet is valid
        if (!checkPacketValidity(packet))  return false;
        
        //decode the packet
        memset(&firings, 0, sizeof(firings));
        decodePacket(packet);
        // find the start of a new revolution
        // if there is one, new_sweep_start will be the index of the start firing,
        // otherwise, new_sweep_start will be FIRINGS_PER_PACKET.
        size_t new_sweep_start = 0;
        do {
            if (abs(firings.azimuth[new_sweep_start] - last_azimuth) > 35900) {
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
            }
        }

        if (use_time_service) {
            memset(&cur_time, 0, sizeof(cur_time));
            cur_time.tm_year = packet->data[year] + 2000 - 1900;
            cur_time.tm_mon  = packet->data[month] - 1;
            cur_time.tm_mday = packet->data[day];
            cur_time.tm_hour = packet->data[1197];
            cur_time.tm_min  = packet->data[1198];
            cur_time.tm_sec  = packet->data[1199];

            packet_time_s = static_cast<uint64_t>(timegm(&cur_time)); //s
            packet_time_ns = (packet->data[1200] +
                             (packet->data[1201] << 8) +
                             (packet->data[1202] << 16) +
                             (packet->data[1203] << 24)) * 1e3; //ns
            timeStamp = ros::Time(packet_time_s, packet_time_ns % 1000000000);
            packet->stamp = timeStamp;
            current_packet_time = timeStamp.toSec();
        } else {
            packet->stamp = ros::Time::now();
            current_packet_time = packet->stamp.toSec();
        }
        
        for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
            //check if the point is valid
            if (!(firings.distance[fir_idx] >= min_range && firings.distance[fir_idx] <= max_range)) continue;
            if (angle_able_max > 36000){
                if((firings.azimuth[fir_idx] > (angle_able_max - 36000)) && firings.azimuth[fir_idx] < angle_able_min) continue;
            } else {
                if((firings.azimuth[fir_idx] > angle_able_max ) || firings.azimuth[fir_idx] < angle_able_min) continue;
            }

            //填充laserscan距离 强度内容
            if (coordinate_opt) {
                float horizontal_angle = firings.azimuth[fir_idx] * 0.01f * DEG_TO_RAD;
                uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                scan_msg->ranges[point_size - point_index - 1] = firings.distance[fir_idx];
                scan_msg->intensities[point_size - point_index - 1] = firings.intensity[fir_idx];
            } else {
                float h_angle = (45000.0 - firings.azimuth[fir_idx]) < 36000.0 ?
                                 45000.0 - firings.azimuth[fir_idx] : 9000.0 - firings.azimuth[fir_idx];
                float horizontal_angle = h_angle * 0.01f * DEG_TO_RAD;
                uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                scan_msg->ranges[point_index] = firings.distance[fir_idx];
                scan_msg->intensities[point_index] = firings.intensity[fir_idx];
            }
        }
        // a new sweep begins ----------------------------------------------------
        if (end_fir_idx != SCANS_PER_PACKET) {
            //publish Last frame scan
            if (last_packet_time > 1e-6) {
                sweep_end_time = packet->stamp.toSec() - (current_packet_time - last_packet_time) * 
                                (SCANS_PER_PACKET - end_fir_idx) / SCANS_PER_PACKET;
            } else {
                sweep_end_time = current_packet_time;
            }

            sweep_end_time = sweep_end_time > 0 ? sweep_end_time : 0;

            {
                std::unique_lock<std::mutex> lock(laserscan_lock);
                scan_msg_bak = std::move(scan_msg);
            }
            // 发布当前一圈数据
            threadPool_->enqueue([this]() { publishScan(); });
            
            scan_msg.reset(new sensor_msgs::LaserScan);
            scan_msg->angle_min = 0;
            scan_msg->angle_max = M_PI * 2;
            scan_msg->range_min = min_range;
            scan_msg->range_max = max_range;
            scan_msg->angle_increment = horizontal_angle_resolution * DEG_TO_RAD;
            point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

            scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
            //prepare the next frame scan
            //每帧最后一个数据包有下一圈的点。填充下一圈的点
            last_azimuth = firings.azimuth[SCANS_PER_PACKET - 1];
            start_fir_idx = end_fir_idx;
            end_fir_idx = SCANS_PER_PACKET;
            for (size_t fir_idx = start_fir_idx; fir_idx < end_fir_idx; ++fir_idx) {
                //check if the point is valid
                if (!(firings.distance[fir_idx] >= min_range && firings.distance[fir_idx] <= max_range)) continue;
                if (angle_able_max > 36000){
                    if((firings.azimuth[fir_idx] > (angle_able_max - 36000)) && firings.azimuth[fir_idx] < angle_able_min) continue;
                } else {
                    if((firings.azimuth[fir_idx] > angle_able_max ) || firings.azimuth[fir_idx] < angle_able_min) continue;
                }
                
                if (coordinate_opt) {
                    float horizontal_angle = firings.azimuth[fir_idx] * 0.01f * DEG_TO_RAD;
                    uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                    scan_msg->ranges[point_size - point_index - 1] = firings.distance[fir_idx];
                    scan_msg->intensities[point_size - point_index - 1] = firings.intensity[fir_idx];
                } else {
                    float h_angle = (45000.0 - firings.azimuth[fir_idx]) < 36000.0 ?
                                     45000.0 - firings.azimuth[fir_idx] : 9000.0 - firings.azimuth[fir_idx];

                    float horizontal_angle = h_angle * 0.01f * DEG_TO_RAD;

                    uint point_index = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_index = (point_index < point_size) ? point_index : (point_index % point_size);
                    scan_msg->ranges[point_index] = firings.distance[fir_idx];
                    scan_msg->intensities[point_index] = firings.intensity[fir_idx];
                }
            }
        }

        last_packet_time = current_packet_time;
        return true;
    }
}  // namespace lslidar_driver
