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

#include <lslidar_driver/lslidar_ls_driver.hpp>

namespace lslidar_driver {
    LslidarLsDriver::LslidarLsDriver(ros::NodeHandle &node, ros::NodeHandle &private_nh) : LslidarDriver(node, private_nh),
                                                                                           nh(node),
                                                                                           pnh(private_nh),
                                                                                           packet_end_time(0.0),
                                                                                           current_packet_time(0.0),
                                                                                           last_packet_time(0.0),
                                                                                           return_mode(1),
                                                                                           g_fAngleAcc_V(0.01),
                                                                                           packet_loss(false),
                                                                                           get_ms06_param(true),
                                                                                           is_add_frame_(false),
                                                                                           last_packet_number_(-1), 
                                                                                           total_packet_loss_(0),
                                                                                           point_cloud_xyzirt_(new pcl::PointCloud<VPoint>),
                                                                                           point_cloud_xyzirt_bak_(new pcl::PointCloud<VPoint>),
                                                                                           point_cloud_xyzirt_pub_(new pcl::PointCloud<VPoint>) {
        services_ = std::make_shared<LslidarLsServices>();
    }

    bool LslidarLsDriver::loadParameters() {
        pnh.param<std::string>("pcap", dump_file, std::string(""));
        pnh.param<double>("packet_rate", packet_rate, 15000.0);
        pnh.param<std::string>("frame_id", frame_id, "laser_link");
        pnh.param<bool>("add_multicast", add_multicast, false);
        pnh.param<std::string>("group_ip", group_ip_string, std::string("224.1.1.2"));
        pnh.param<bool>("use_time_service", use_time_service, false);
        pnh.param<std::string>("device_ip", lidar_ip_string, std::string("192.168.1.200"));
        pnh.param<int>("msop_port", msop_udp_port, (int) MSOP_DATA_PORT_NUMBER);
        pnh.param<int>("difop_port", difop_udp_port, (int) DIFOP_DATA_PORT_NUMBER);
        pnh.param<std::string>("pointcloud_topic", pointcloud_topic_, "lslidar_point_cloud");
        pnh.param<bool>("use_first_point_time", use_first_point_time, false);
        pnh.param<bool>("use_absolute_time", use_absolute_time, false);
        pnh.param<double>("min_range", min_range, 0.5);
        pnh.param<double>("max_range", max_range, 300.0);
        pnh.param<bool>("is_pretreatment", is_pretreatment, false);
        pnh.param<double>("x_offset", x_offset, 0.0);
        pnh.param<double>("y_offset", y_offset, 0.0);
        pnh.param<double>("z_offset", z_offset, 0.0);
        pnh.param<double>("roll", roll, 0.0);
        pnh.param<double>("pitch", pitch, 0.0);
        pnh.param<double>("yaw", yaw, 0.0);
        
        pnh.param<int>("scan_start_angle", scan_start_angle, -6000);
        pnh.param<int>("scan_end_angle", scan_end_angle, 6000);
        pnh.param<bool>("packet_loss", packet_loss, false);
        pnh.param<std::string>("lidar_model", lidar_model, "LSS3");

        bool is_add_frame_tmp = false;
        pnh.param<bool>("is_add_frame", is_add_frame_tmp, false);
        is_add_frame_.store(is_add_frame_tmp); 

        ROS_INFO("Using time service or not: %d", use_time_service);
        ROS_INFO("Is packet loss detection enabled: %d", packet_loss);

        ROS_INFO_STREAM("Only accepting packets from IP address: " << lidar_ip_string.c_str());
        if (add_multicast) ROS_INFO_STREAM("Opening UDP socket: group_address " << group_ip_string);

        return true;
    }

    bool LslidarLsDriver::createRosIO() {
        pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic_, 10);
        fault_code_pub_ = nh.advertise<std_msgs::String>("lslidar_fault_code", 1);
        lidar_info_pub_ = nh.advertise<lslidar_msgs::LslidarInformation>("lslidar_device_info", 1);
        if (packet_loss) packet_loss_pub_ = nh.advertise<std_msgs::Int64>("packet_loss", 10);
        time_pub_ = nh.advertise<std_msgs::Float64>("time_topic", 10);

        angle_distortion_correction_service_ = nh.advertiseService("angle_distortion_correction", &LslidarLsServices::setAngleDistortionCorrection,
                                                std::dynamic_pointer_cast<LslidarLsServices>(services_).get());
        network_config_service_ = nh.advertiseService("network_setup", &LslidarLsServices::setIpAndPort,
                                                std::dynamic_pointer_cast<LslidarLsServices>(services_).get());
        time_mode_service_ = nh.advertiseService("time_mode", &LslidarLsServices::setTimeMode, 
                                                std::dynamic_pointer_cast<LslidarLsServices>(services_).get());
        frame_rate_service_ = nh.advertiseService("frame_rate", &LslidarLsServices::setFrameRate, 
                                                std::dynamic_pointer_cast<LslidarLsServices>(services_).get());
        invalid_data_service_ = nh.advertiseService("invalid_data", &LslidarLsServices::setInvalidData, 
                                                std::dynamic_pointer_cast<LslidarLsServices>(services_).get());
        standby_mode_service_ = nh.advertiseService("standby_mode", &LslidarLsServices::setStandbyMode, 
                                                std::dynamic_pointer_cast<LslidarLsServices>(services_).get());
        
        if (!dump_file.empty()) {
            msop_input_.reset(new lslidar_driver::InputPCAP(pnh, msop_udp_port, 1206, packet_rate, dump_file));
            difop_input_.reset(new lslidar_driver::InputPCAP(pnh, difop_udp_port, 1206, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_driver::InputSocket(pnh, msop_udp_port, 1206));
            difop_input_.reset(new lslidar_driver::InputSocket(pnh, difop_udp_port, 1206));
        }

        thread_pool_->enqueue([this]() { difopPoll(); });
        
        return true;
    }

    void LslidarLsDriver::initTimeStamp() {
        for (unsigned char &i : this->packetTimeStamp) {
            i = 0;
        }
        this->pointcloudTimeStamp = 0;

        point_time_offset = use_first_point_time ? 2 : 0;
        relative_time_offset = use_absolute_time ? 0 : 1;
    }

    bool LslidarLsDriver::initialize() {
        if (!loadParameters()) {
            ROS_ERROR("Cannot load all required ROS parameters...");
            return false;
        }

        if (!createRosIO()) {
            ROS_ERROR("Cannot create all ROS IO...");
            return false;
        }

        if (!getLidarInformation()) {
            ROS_ERROR("Cannot to obtain lidar configuration...");
            return false;
        }

        this->initTimeStamp();

        if (is_pretreatment) {
            pointcloud_transform_.setTransform(x_offset, y_offset, z_offset, roll, pitch, yaw);
        }

        // create the sin and cos table for different azimuth and vertical values
        for (int j = 0; j < 36000; ++j) {
            double angle = static_cast<double>(j) * 0.01 * 0.017453293;
            sin_table[j] = sin(angle);
            cos_table[j] = cos(angle);
        }

        double mirror_angle_s3[4] = {1.5, -0.5, 0.5, -1.5};   //摆镜角度   //根据通道不同偏移角度不同
        double mirror_angle_s4[8] = {-2.555, -1.825, -1.095, -0.365, 0.365, 1.095, 1.825, 2.555};

        if (lidar_model == "LSS3") {
            channel_number_shift = CHANNEL_SHIFT_S3;
            symbol_shift = SYMBOL_SHIFT_S3;
            angle_v_mask = ANGLE_V_MASK_S3;
            angle_h_mask = ANGLE_H_MASK_S3;

            m_offset = m_offset_s3;
            cos1 = cos30;
            sin1 = sin30;
            sin2 = sin60;
            for (int i = 0; i < 4; ++i) {
                cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle_s3[i]));
                sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle_s3[i]));
            }
        } else if (lidar_model == "LSS4") {
            channel_number_shift = CHANNEL_SHIFT_S4;
            symbol_shift = SYMBOL_SHIFT_S4;
            angle_v_mask = ANGLE_V_MASK_S4;
            angle_h_mask = ANGLE_H_MASK_S4;

            m_offset = m_offset_s4;
            cos1 = cos45;
            sin1 = sin45;
            sin2 = sin90;
            for (int i = 0; i < 8; ++i) {
                cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle_s4[i]));
                sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle_s4[i]));
            }
        }

        return true;
    }

    void LslidarLsDriver::difopPoll() {
        lslidar_msgs::LslidarPacketPtr difop_packet(new lslidar_msgs::LslidarPacket());
        static bool stack_frames = true;

        while (ros::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc > 1) {
                if (difop_packet->data[0] == 0x00 || difop_packet->data[0] == 0xa5) {
                    if (difop_packet->data[1] == 0xff && difop_packet->data[2] == 0x00 &&
                        difop_packet->data[3] == 0x5a) {
                       
                        if (lidar_model == "LSS3" && stack_frames) {
                            // LS320 LS400
                            if (difop_packet->data[231] == 64 || difop_packet->data[231] == 65) {   
                                is_add_frame_.store(true);
                            }

                            stack_frames = false;
                        }

                        if (lidar_model == "LSS4") {
                            short distortion_angle = ((difop_packet->data[238] << 8) + difop_packet->data[239]);

                            if (0xffff != distortion_angle && 0 != distortion_angle) {
                                m_offset = distortion_angle * 0.01;
                            }
                        }

                        // 服务配置雷达传递设备包
                        services_->getDifopPacket(difop_packet);

                        m_horizontal_point = difop_packet->data[184] * 256 + difop_packet->data[185];

                        int majorVersion = difop_packet->data[1202];
                        int minorVersion1 = difop_packet->data[1203] / 16;
                        int minorVersion2 = difop_packet->data[1203] % 16;

                        //v1.1 :0.01   //v1.2以后  ： 0.0025
                        g_fAngleAcc_V = (1 > majorVersion || (1 == majorVersion && minorVersion1 > 1)) ? 0.0025 : 0.01;

                        std::ostringstream oss;
                        for (int i = 0; i < 4; ++i) {
                            std::bitset<8> bits(difop_packet->data[887 + i]);
                            oss << bits;
                            if (i < 3) oss << ' ';
                        }

                        std_msgs::String fault_code_msg;
                        fault_code_msg.data = oss.str();
                        fault_code_pub_.publish(fault_code_msg);

                        Information device = device_info_->getDeviceInfo(difop_packet);

                        lidar_info_data_ = boost::make_shared<lslidar_msgs::LslidarInformation>();
                        lidar_info_data_->Lidar_IP = device.lidarIp;
                        lidar_info_data_->Destination_IP = device.destinationIP;
                        lidar_info_data_->Lidar_Mac_Address = device.lidarMacAddress;
                        lidar_info_data_->Msop_Port = device.msopPort;
                        lidar_info_data_->Difop_Port = device.difopPort;
                        lidar_info_data_->Lidar_Serial_Number = device.lidarSerialNumber;
                        lidar_info_data_->FPGA_Board_2_Program = device.secondBoardProgram;
                        lidar_info_data_->FPGA_Board_3_Program = device.thirdBoardProgram;
                        lidar_info_pub_.publish(lidar_info_data_);

                        is_get_difop_.store(true);
                    }
                }
            } else if (rc < 0) {
                return;
            }
        }
    }

    void LslidarLsDriver::publishPointCloudNew() {
        if (!is_get_difop_.load()) return;

        std::unique_lock<std::mutex> lock(pc_mutex_);
        point_cloud_xyzirt_pub_->header.frame_id = frame_id;
        point_cloud_xyzirt_pub_->height = 1;

        if (is_pretreatment) pointcloud_transform_.applyTransform(*point_cloud_xyzirt_pub_);

        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(*point_cloud_xyzirt_pub_, pc_msg);
        pc_msg.header.stamp = ros::Time(point_cloud_time);
        pointcloud_pub_.publish(pc_msg);

        std_msgs::Float64 time_msg;
        time_msg.data = point_cloud_time;
        time_pub_.publish(time_msg);
    }

    int LslidarLsDriver::convertCoordinate(const struct FiringLS &lidardata) {    
        double Angle_H = lidardata.azimuth;        
        double Angle_V = lidardata.vertical_angle; 

        if (Angle_H < 0.0) Angle_H += 360.0;
        if (Angle_V < 0.0) Angle_V += 360.0;

        int table_index_H = int(Angle_H * 100) % 36000;
        int table_index_V = int(Angle_V * 100) % 36000;

        const float x_coord = lidardata.distance * cos_table[table_index_V] * sin_table[table_index_H];
        const float y_coord = lidardata.distance * cos_table[table_index_V] * cos_table[table_index_H];
        const float z_coord = lidardata.distance * sin_table[table_index_V];

        VPoint point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.channel_number;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        point_cloud_xyzirt_bak_->points.push_back(point);
        ++point_cloud_xyzirt_bak_->width;

        return 0;
    }

    int LslidarLsDriver::convertCoordinateDistortion(const struct FiringLS &lidardata) {
        double fAngle_H = 0.0;     
        double fAngle_V = 0.0; 
        fAngle_H = lidardata.azimuth;
        fAngle_V = lidardata.vertical_angle;

        double fSinV_angle = 0;
        double fCosV_angle = 0;

        double fGalvanometrtAngle = fAngle_V + m_offset;

        if (fGalvanometrtAngle < 0.0) fGalvanometrtAngle += 360.0;

        if (fAngle_H < 0.0) fAngle_H += 360.0;
        
        int table_index_V = int(fGalvanometrtAngle * 100) % 36000;
        int table_index_H = int(fAngle_H * 100) % 36000;

        double fAngle_R0 = cos1 * cos_mirror_angle[lidardata.channel_number] * cos_table[table_index_V] -
                           sin_table[table_index_V] * sin_mirror_angle[lidardata.channel_number];

        fSinV_angle = 2 * fAngle_R0 * sin_table[table_index_V] + sin_mirror_angle[lidardata.channel_number];
        fCosV_angle = sqrt(1 - pow(fSinV_angle, 2));

        double fSinCite = (2 * fAngle_R0 * cos_table[table_index_V] * sin1 -
                           cos_mirror_angle[lidardata.channel_number] * sin2) / fCosV_angle;
        double fCosCite = sqrt(1 - pow(fSinCite, 2));

        double fSinCite_H = sin_table[table_index_H] * fCosCite + cos_table[table_index_H] * fSinCite;
        double fCosCite_H = cos_table[table_index_H] * fCosCite - sin_table[table_index_H] * fSinCite;

        const float x_coord = lidardata.distance * fCosV_angle * fSinCite_H;
        const float y_coord = lidardata.distance * fCosV_angle * fCosCite_H;
        const float z_coord = lidardata.distance * fSinV_angle;

        VPoint point;
        point.x = x_coord;
        point.y = y_coord;
        point.z = z_coord;
        point.intensity = lidardata.intensity;
        point.ring = lidardata.channel_number;
        point.time = lidardata.time;
        point_cloud_xyzirt_->points.push_back(point);
        ++point_cloud_xyzirt_->width;

        point_cloud_xyzirt_bak_->points.push_back(point);
        ++point_cloud_xyzirt_bak_->width;

        return 0;
    }

    bool LslidarLsDriver::poll() {
        lslidar_msgs::LslidarPacketPtr packet(new lslidar_msgs::LslidarPacket());

        while (ros::ok()) {
            // keep reading until full packet received
            int rc = msop_input_->getPacket(packet);
            if (rc > 1) break;       // got a full packet?
            if (rc == 0) continue;    // No full packet, retry
            if (rc < 0) return false; // System-level error, terminate,end of file reached?
        }

        // publish message using time of last packet read
        if (use_time_service) {
            if (0xff == packet->data[1194]) {    //ptp授时
                uint64_t timestamp_s = (packet->data[1195] * 0 + (packet->data[1196] << 24) +
                                        (packet->data[1197] << 16) + (packet->data[1198] << 8) + packet->data[1199] * pow(2, 0));
                uint64_t timestamp_nsce = (packet->data[1200] << 24) + (packet->data[1201] << 16) +
                                          (packet->data[1202] << 8) + (packet->data[1203]);
                current_packet_time = ros::Time(timestamp_s, timestamp_nsce).toSec();
            } else {          //gps授时
                this->packetTimeStamp[4] = packet->data[1199];
                this->packetTimeStamp[5] = packet->data[1198];
                this->packetTimeStamp[6] = packet->data[1197];
                this->packetTimeStamp[7] = packet->data[1196];
                this->packetTimeStamp[8] = packet->data[1195];
                this->packetTimeStamp[9] = packet->data[1194];
                struct tm cur_time{};
                memset(&cur_time, 0, sizeof(cur_time));
                cur_time.tm_sec = this->packetTimeStamp[4];
                cur_time.tm_min = this->packetTimeStamp[5];
                cur_time.tm_hour = this->packetTimeStamp[6];
                cur_time.tm_mday = this->packetTimeStamp[7];
                cur_time.tm_mon = this->packetTimeStamp[8] - 1;
                cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
                this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time)); //s
                uint64_t packet_timestamp;
                packet_timestamp = packet->data[1203] +
                                   (packet->data[1202] << 8) +
                                   (packet->data[1201] << 16) +
                                   (packet->data[1200] << 24); //ns
                current_packet_time = ros::Time(this->pointcloudTimeStamp, packet_timestamp).toSec();
            }
        } else {
            current_packet_time = ros::Time::now().toSec();
        }

        lslidarPacketProcess(packet);

        return true;
    }

    void LslidarLsDriver::packetProcessSingle(const lslidar_msgs::LslidarPacketPtr& msg) {
        FiringLS lidardata;
        bool packetType = false;
        if (packet_loss) checkPacketLoss(msg, 1192, 2);
        double point_interval_time = (current_packet_time - last_packet_time) * SINGLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < POINTS_PER_PACKET_SINGLE_ECHO; point_idx += 8, ++point_num) {
            if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) && (msg->data[point_idx + 2] == 0xbb) && 
                (msg->data[point_idx + 3] == 0xcc) && (msg->data[point_idx + 4] == 0xdd)) {   // 帧头判断
                point_cloud_timestamp = last_packet_time + point_interval_time * (point_num + point_time_offset);
                point_cloud_time = use_first_point_time 
                    ? (is_add_frame_.load() ? first_two_point_cloud_time : last_point_cloud_time)
                    : point_cloud_timestamp;

                first_two_point_cloud_time = last_point_cloud_time;
                last_point_cloud_time = point_cloud_timestamp;

                packetType = true;
                frame_count++;
            } else {    // 预计算点的距离
                double point_distance = ((msg->data[point_idx + 4] << 16) + (msg->data[point_idx + 5] << 8) +
                                          msg->data[point_idx + 6]) * g_fDistanceAcc;
                if (point_distance < min_range || point_distance > max_range) continue;
                memset(&lidardata, 0, sizeof(lidardata));

                // 水平角度
                int fAngle_H = (msg->data[point_idx] << 8) + msg->data[point_idx + 1];
                if (fAngle_H > 32767) {
                    fAngle_H = (fAngle_H - 65536);
                }
                if ((fAngle_H < scan_start_angle) || (fAngle_H > scan_end_angle)) continue;
                lidardata.azimuth = fAngle_H * 0.01;

                // 垂直角度+通道号
                int iTempAngle = msg->data[point_idx + 2];
                int iChannelNumber = iTempAngle >> channel_number_shift;
                int iSymmbol = (iTempAngle >> symbol_shift) & 0x01;
                double fAngle_V = 0.0;
                if (1 == iSymmbol) { // 符号位 0：正数 1：负数
                    int iAngle_V = (msg->data[point_idx + 2] << 8) + msg->data[point_idx + 3];
                    fAngle_V = iAngle_V | angle_v_mask;
                    if (fAngle_V > 32767) {
                        fAngle_V = (fAngle_V - 65536);
                    }
                } else {
                    int iAngle_Hight = iTempAngle & angle_h_mask;
                    fAngle_V = (iAngle_Hight << 8) + msg->data[point_idx + 3];
                }

                lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                lidardata.channel_number = iChannelNumber;
                lidardata.distance = point_distance;
                lidardata.intensity = msg->data[point_idx + 7];
                lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp  * relative_time_offset;
                lidarConvertCoordinate(lidardata);  // 计算坐标
            }

            prepareAndPublishPointCloud(packetType);
        }

        last_packet_time = current_packet_time;
    }

    void LslidarLsDriver::packetProcessDouble(const lslidar_msgs::LslidarPacketPtr &msg) {
        FiringLS lidardata;
        bool packetType = false;
        if (packet_loss) checkPacketLoss(msg, 1188, 6);
        double point_interval_time = (current_packet_time - last_packet_time) * DOUBLE_ECHO;

        for (size_t point_idx = 0, point_num = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 12, ++point_num) {
            if ((msg->data[point_idx] == 0xff) && (msg->data[point_idx + 1] == 0xaa) && (msg->data[point_idx + 2] == 0xbb) && 
                (msg->data[point_idx + 3] == 0xcc) && (msg->data[point_idx + 4] == 0xdd)) {
                point_cloud_timestamp = last_packet_time + point_interval_time * (point_num + point_time_offset);
                point_cloud_time = use_first_point_time 
                    ? (is_add_frame_.load() ? first_two_point_cloud_time : last_point_cloud_time)
                    : point_cloud_timestamp;

                first_two_point_cloud_time = last_point_cloud_time;
                last_point_cloud_time = point_cloud_timestamp;

                packetType = true;
                frame_count++;
            } else {
                double point_distance1 = ((msg->data[point_idx + 4] << 16) + (msg->data[point_idx + 5] << 8) +
                                           msg->data[point_idx + 6]) * g_fDistanceAcc;
                if (point_distance1 < min_range || point_distance1 > max_range) continue;
                double point_distance2 = ((msg->data[point_idx + 8] << 16) + (msg->data[point_idx + 9] << 8) +
                                           msg->data[point_idx + 10]) * g_fDistanceAcc;
                memset(&lidardata, 0, sizeof(lidardata));
                //水平角度
                int fAngle_H = (msg->data[point_idx] << 8) + msg->data[point_idx + 1];
                if (fAngle_H > 32767) {
                    fAngle_H = (fAngle_H - 65536);
                }
                if ((fAngle_H < scan_start_angle) || (fAngle_H > scan_end_angle)) continue;
                //垂直角度+通道号
                int iTempAngle = msg->data[point_idx + 2];
                int iChannelNumber = iTempAngle >> channel_number_shift; //左移六位 通道号
                int iSymmbol = (iTempAngle >> symbol_shift) & 0x01; //左移五位 符号位
                double fAngle_V = 0.0;
                if (1 == iSymmbol) // 符号位 0：正数 1：负数
                {
                    int iAngle_V = (msg->data[point_idx + 2] << 8) + msg->data[point_idx + 3];

                    fAngle_V = iAngle_V | angle_v_mask;
                    if (fAngle_V > 32767) {
                        fAngle_V = (fAngle_V - 65536);
                    }
                } else {
                    int iAngle_Hight = iTempAngle & angle_h_mask;
                    fAngle_V = (iAngle_Hight << 8) + msg->data[point_idx + 3];
                }

                lidardata.azimuth = fAngle_H * 0.01;
                lidardata.vertical_angle = fAngle_V * g_fAngleAcc_V;
                lidardata.channel_number = iChannelNumber;
                lidardata.distance = point_distance1;
                lidardata.intensity = msg->data[point_idx + 7];
                lidardata.time = last_packet_time + point_interval_time * (point_num + 1) - point_cloud_timestamp * relative_time_offset;
                lidarConvertCoordinate(lidardata);  // 第一个点

                lidardata.distance = point_distance2;
                if (lidardata.distance < min_range || lidardata.distance > max_range) continue;
                lidardata.intensity = msg->data[point_idx + 11];
                lidarConvertCoordinate(lidardata);  // 第二个点
            }

            prepareAndPublishPointCloud(packetType);
        }

        last_packet_time = current_packet_time;
    }

    void LslidarLsDriver::prepareAndPublishPointCloud(bool& packetType) {
        if (packetType) {
            if (is_add_frame_.load()) {
                if (frame_count >= 2) {
                    {
                        std::unique_lock<std::mutex> lock(pc_mutex_);
                        point_cloud_xyzirt_pub_ = std::move(point_cloud_xyzirt_);
                    }
                    thread_pool_->enqueue([this]() { publishPointCloudNew(); });
                }
                packetType = false;
                point_cloud_xyzirt_ = std::move(point_cloud_xyzirt_bak_);
                point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
            } else {
                {
                    std::unique_lock<std::mutex> lock(pc_mutex_);
                    point_cloud_xyzirt_pub_ = std::move(point_cloud_xyzirt_);
                }
                thread_pool_->enqueue([this]() { publishPointCloudNew(); });
                packetType = false;
                point_cloud_xyzirt_.reset(new pcl::PointCloud<VPoint>);
                point_cloud_xyzirt_bak_.reset(new pcl::PointCloud<VPoint>);
            }
        }
    }

    void LslidarLsDriver::checkPacketLoss(const lslidar_msgs::LslidarPacketPtr &msg, int data_offset, int byte_count) {
        int64_t current_packet_number_ = 0;

        for (int i = 0; i < byte_count; ++i) {
            current_packet_number_ = (current_packet_number_ << 8) + msg->data[data_offset + i];
        }

        tmp_packet_number_ = current_packet_number_;

        if(current_packet_number_ - last_packet_number_ < 0){current_packet_number_ += 65536;}

        if (current_packet_number_ - last_packet_number_ > 1  && last_packet_number_ != -1) {
            // if(current_packet_number_ - last_packet_number_ > 20) ROS_WARN("num: %ld  98-03: %02x %02x %02x %02x %02x %02x", current_packet_number_ - last_packet_number_ ,msg->data[1198],msg->data[1199],msg->data[1200],msg->data[1201],msg->data[1202],msg->data[1203]);
            total_packet_loss_ += current_packet_number_ - last_packet_number_ - 1;
            std_msgs::Int64 loss_data;
            loss_data.data = total_packet_loss_;
            packet_loss_pub_.publish(loss_data);
        }
        last_packet_number_ = tmp_packet_number_;
    }

    bool LslidarLsDriver::getLidarInformation(){
        lslidar_msgs::LslidarPacketPtr msg(new lslidar_msgs::LslidarPacket());
    
        while (ros::ok()) {
            // keep reading until full packet received
            int rc = msop_input_->getPacket(msg);
            if (rc > 1) break;       // got a full packet?
            if (rc == 0) continue;
            if (rc < 0) return false; // end of file reached?
        }
        
        if(get_ms06_param && m_horizontal_point != 0 && msg->data[1204] == 192){
            //ms06  param
            double mirror_angle[4] = {1.5, 0.5, -0.5, -1.5};
            for (int i = 0; i < 4; ++i) {
                cos_mirror_angle[i] = cos(DEG2RAD(mirror_angle[i]));
                sin_mirror_angle[i] = sin(DEG2RAD(mirror_angle[i]));
            }
            m_offset = 10.82;
            g_fAngleAcc_V = 0.01;
            g_fDistanceAcc = 0.004;
            get_ms06_param = false;
            ROS_INFO("Lidar model MS06");
        }

        if(msg->data[1205] == 0x01 || msg->data[1205] == 0x11) {
            lslidarPacketProcess = std::bind(&LslidarLsDriver::packetProcessSingle, this, std::placeholders::_1);
            if (msg->data[1205] == 0x01) {
                lidarConvertCoordinate = std::bind(&LslidarLsDriver::convertCoordinateDistortion, this, std::placeholders::_1);
                ROS_INFO("Lidar distortion correction off");
            } else if (msg->data[1205] == 0x11) {
                lidarConvertCoordinate = std::bind(&LslidarLsDriver::convertCoordinate, this, std::placeholders::_1);
                ROS_INFO("Lidar distortion correction on");
            }
            ROS_INFO("Lidar echo mode: single echo");
        } else if(msg->data[1205] == 0x02 || msg->data[1205] == 0x12){
            lslidarPacketProcess = std::bind(&LslidarLsDriver::packetProcessDouble, this, std::placeholders::_1);
            if (msg->data[1205] == 0x02) {
                lidarConvertCoordinate = std::bind(&LslidarLsDriver::convertCoordinateDistortion, this, std::placeholders::_1);
                ROS_INFO("Lidar distortion correction off");
            } else if (msg->data[1205] == 0x12) {
                lidarConvertCoordinate = std::bind(&LslidarLsDriver::convertCoordinate, this, std::placeholders::_1);
                ROS_INFO("Lidar distortion correction on");
            }
            ROS_INFO("Lidar echo mode: double echo");
        } else {
            return false;
        }

        ROS_INFO("Lidar model: %s", lidar_model.c_str());

        return true;
    };

} // namespace lslidar_driver
