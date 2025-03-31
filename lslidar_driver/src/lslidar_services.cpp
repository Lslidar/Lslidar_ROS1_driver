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

#include "lslidar_driver/lslidar_services.hpp"

namespace lslidar_driver {
    // 获取设备包
    int LslidarServices::getDifopPacket(lslidar_msgs::LslidarPacketPtr &pkt) {
        if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A) {
            
            {
                std::lock_guard<std::mutex> lock(difop_data_mutex);
                mempcpy(difop_data, pkt->data.data(), 1206);
            }

            lslidar_ip = std::to_string(difop_data[10]) + "." + std::to_string(difop_data[11]) + "." +
                         std::to_string(difop_data[12]) + "." + std::to_string(difop_data[13]);

            difop_valid.store(true);
            return 0;
        } else {
            ROS_ERROR("Can not get dev packet! Set failed !!!");
            difop_valid.store(false);
            return -1;
        }
    }

    void LslidarServices::setUcwpPacketHeader(unsigned char *ucwp_data) {
        ucwp_data[0] = 0xAA;
        ucwp_data[1] = 0x00;
        ucwp_data[2] = 0xFF;
        ucwp_data[3] = 0x11;
        ucwp_data[4] = 0x22;
        ucwp_data[5] = 0x22;
        ucwp_data[6] = 0xAA;
        ucwp_data[7] = 0xAA;
    }

    bool LslidarServices::setUcwpData(unsigned char *ucwp_data) {
        if (!difop_valid.load()) {
            ROS_ERROR("No valid Difop packet available. Set failed!");
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(difop_data_mutex);
            mempcpy(ucwp_data, difop_data, 1206);
        }

        difop_valid.store(false);
        setUcwpPacketHeader(ucwp_data);

        return true;
    }

    bool LslidarServices::sendUcwpPacketTolidar(unsigned char *ucwp_data) const {
        int socketid;
        sockaddr_in addrSrv{};
        socketid = socket(2, 2, 0);
        addrSrv.sin_addr.s_addr = inet_addr(lslidar_ip.c_str());
        addrSrv.sin_family = AF_INET;
        addrSrv.sin_port = htons(2368);
        sendto(socketid, (const char *) ucwp_data, 1206, 0, (struct sockaddr *) &addrSrv, sizeof(addrSrv));
        close(socketid);
        
        return true;
    }

    bool LslidarServices::checkAndSetIp(const std::string& ip, unsigned char* ucwp_data, int index, bool is_lidar_ip) {
        // 检查IP
        unsigned short first_value, second_value, third_value, end_value;
        if (sscanf(ip.c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value)!= 4) {
            return false;
        }

        std::string disable_ip;
        if (is_lidar_ip) {
            disable_ip = std::to_string(ucwp_data[14]) + "." + std::to_string(ucwp_data[15]) + "." +
                         std::to_string(ucwp_data[16]) + "." + std::to_string(ucwp_data[17]);
            if (first_value >= 224) {
                ROS_ERROR("Parameter error, please check the input LiDAR ip parameters.");
                return false;
            }
        } else {
            disable_ip = std::to_string(ucwp_data[10]) + "." + std::to_string(ucwp_data[11]) + "." +
                         std::to_string(ucwp_data[12]) + "." + std::to_string(ucwp_data[13]);
            if (first_value >= 240) {
                ROS_ERROR("Parameter error, please check the input destination ip parameters.");
                return false;
            }
        }

        if (first_value == 0 || first_value == 127 || first_value >= 255 || disable_ip == ip) {
            ROS_ERROR("Parameter error, please check the input ip parameters. %s", ip.c_str());
            return false;
        }

        // 设置IP
        ucwp_data[index] = first_value;
        ucwp_data[index + 1] = second_value;
        ucwp_data[index + 2] = third_value;
        ucwp_data[index + 3] = end_value;

        return true;
    }

    bool LslidarServices::checkAndSetPort(int port, unsigned char* ucwp_data, int index, bool is_data_port) {
        // 检查端口
        int disable_port = ucwp_data[index] * 256 + ucwp_data[index];

        if (port < 1025 || port > 65535 || port == disable_port) {
            ROS_ERROR("Parameter error, please check the input port parameters.");
            return false;
        }

        // 设置端口
        if (is_data_port) {
            ucwp_data[24] = port / 256;
            ucwp_data[25] = port % 256;
        } else {
            ucwp_data[26] = port / 256;
            ucwp_data[27] = port % 256;
        }
        
        return true;
    }

    bool LslidarServices::loadConfigFromYAML() {
        std::string config_file_path = ros::package::getPath("lslidar_driver") + "/params/network_setup.yaml";
        try {
            YAML::Node filter_config = YAML::LoadFile(config_file_path);
            LiDAR.lidar_ip = filter_config["lslidar"]["lidar_ip"].as<std::string>();
            LiDAR.destination_ip = filter_config["lslidar"]["destination_ip"].as<std::string>();
            LiDAR.data_port = filter_config["lslidar"]["data_port"].as<uint16_t>();
            LiDAR.dev_port = filter_config["lslidar"]["dev_port"].as<uint16_t>();
            return true;
        } catch (const YAML::Exception& e) {
            ROS_ERROR("Error reading YAML file: %s", e.what());
            return false;
        }
    }

    bool LslidarServices::setIpAndPort(lslidar_msgs::IpAndPort::Request &req,
                                           lslidar_msgs::IpAndPort::Response &res) {
        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        if (req.lidar_ip.empty() && req.destination_ip.empty() && req.data_port == 0 && req.dev_port == 0) {
            if (loadConfigFromYAML()) {
                // 使用yaml中参数配置雷达
                req.lidar_ip = LiDAR.lidar_ip;
                req.destination_ip = LiDAR.destination_ip;
                req.data_port = LiDAR.data_port;
                req.dev_port = LiDAR.dev_port;
                ROS_INFO("Using configuration file parameters.");
            } else {
                ROS_WARN("Failed to read configuration file, please use display parameter transfer.");
                res.result = false;
                return false;
            }
        }

        // 检查并设置雷达IP
        if (!checkAndSetIp(req.lidar_ip, ucwp_data, 10, true)) {
            res.result = false;
            return false;
        }

        // 检查并设置雷达目的IP
        if (!checkAndSetIp(req.destination_ip, ucwp_data, 14, false)) {
            res.result = false;
            return false;
        }

        // 检查并设置雷达目的数据端口
        if (!checkAndSetPort(req.data_port, ucwp_data, 26, true)) {
            res.result = false;
            return false;
        }

        // 检查并设置雷达目的设备端口
        if (!checkAndSetPort(req.dev_port, ucwp_data, 24, false)) {
            res.result = false;
            return false;
        }

        sendUcwpPacketTolidar(ucwp_data);

        ROS_INFO("Successfully! lidar_ip: %s, destination_ip: %s, data_port: %d, dev_port: %d",
                req.lidar_ip.c_str(), req.destination_ip.c_str(), req.data_port, req.dev_port);
        ROS_INFO("Please restart the ROS driver.");
        res.result = true;

        return true;
    }

    bool LslidarServices::setMotorSpeed(lslidar_msgs::MotorSpeed::Request &req,
                                            lslidar_msgs::MotorSpeed::Response &res) {
        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        if (req.motor_speed == 5) {
            ucwp_data[8] = 0x01;
            ucwp_data[9] = 0x2c;
        } else if (req.motor_speed == 10) {
            ucwp_data[8] = 0x02;
            ucwp_data[9] = 0x58;
        } else if (req.motor_speed == 20) {
            ucwp_data[8] = 0x04;
            ucwp_data[9] = 0xB0;
        } else {
            ROS_ERROR("Parameter error, please check the input parameters.");
            res.result = false;
            return false;
        }

        sendUcwpPacketTolidar(ucwp_data);
        ROS_INFO("Successfully set the LiDAR speed to %d.", (ucwp_data[8] << 8) | ucwp_data[9]);
        res.result = true;

        return true;
    }

    void LslidarServices::setTimeModeBytes(unsigned char* ucwp_data, int time_mode) {
        ucwp_data[44] = static_cast<unsigned char>(time_mode);
    }

    std::string LslidarServices::getTimeModeString(int time_mode) {
        switch (time_mode) {
            case 0:
                return "GPS";
            case 1:
                return "PTP L2";
            case 2:
                return "NTP";
            case 3:
                return "PTP UDPv4";
            case 4:
                return "E2E L2";
            case 5:
                return "E2E UDPv4";
            default:
                return "Unknown";
        }
    }

    bool LslidarServices::setTimeMode(lslidar_msgs::TimeMode::Request &req,
                                          lslidar_msgs::TimeMode::Response &res) {
        if(req.time_mode > 5) {
            ROS_ERROR("Parameter error, please check the input parameters.");
            res.result = false;
            return false;
        }

        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        setTimeModeBytes(ucwp_data, req.time_mode);

        if (req.time_mode == 2) {
            std::string ntp_ip = req.ntp_ip;
            std::regex ipv4(
                    "\\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\\b");
            if (!regex_match(ntp_ip, ipv4)) {
                ROS_ERROR("Parameter error, please check the input parameters.");
                res.result = false;
                return true;
            }
            unsigned short first_value, second_value, third_value, end_value;
            sscanf(ntp_ip.c_str(), "%hu.%hu.%hu.%hu", &first_value, &second_value, &third_value, &end_value);
            ucwp_data[28] = first_value;
            ucwp_data[29] = second_value;
            ucwp_data[30] = third_value;
            ucwp_data[31] = end_value;
        }

        sendUcwpPacketTolidar(ucwp_data);

        std::string time_mode_str = getTimeModeString(req.time_mode);
        ROS_INFO("%s mode setting successful!", time_mode_str.c_str());
        res.result = true;

        return true;
    }



    /////////////////////////////// 机械式 ///////////////////////////////
    bool LslidarCxServices::setIpAndPort(lslidar_msgs::IpAndPort::Request &req,
                                             lslidar_msgs::IpAndPort::Response &res) {

        return LslidarServices::setIpAndPort(req, res);
    }

    bool LslidarCxServices::setMotorControl(lslidar_msgs::MotorControl::Request &req,
                                                lslidar_msgs::MotorControl::Response &res) {
        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        if (req.motor_control == 1) {
            ucwp_data[8] = 0x02; 
            ucwp_data[9] = 0x58;
            ucwp_data[41] = 0x00;
        } else if (req.motor_control == 0) {
            ucwp_data[41] = 0x01;
        } else {
            ROS_ERROR("Parameter error, please check the input parameters.");
            res.result = false;
            return false;
        }
        
        sendUcwpPacketTolidar(ucwp_data);
        std::string status = (req.motor_control == 1)? "rotation" : "stopped";
        ROS_INFO("Set successfully! LiDAR %s.", status.c_str());
        res.result = true;

        return true;
    }

    bool LslidarCxServices::setMotorSpeed(lslidar_msgs::MotorSpeed::Request &req,
                                              lslidar_msgs::MotorSpeed::Response &res) {

        return LslidarServices::setMotorSpeed(req, res);
    }

    int LslidarCxServices::GetCxFpgaVersion(unsigned char* ucwp_data) {
        int fpga_version = 0;
        if (ucwp_data[1196] == 0x03 || (ucwp_data[1196] == 0x02 && ucwp_data[1197] >> 4 == 8)) {
            fpga_version = 3;
        } else if ((ucwp_data[1202] >= 0x64) && (ucwp_data[1203] & 0xF0) == 5){
            fpga_version = 5;
        } else if ((ucwp_data[1198] & 0x0F) == 4){
            fpga_version = 4;
        }

        return fpga_version;
    }

    bool LslidarCxServices::setPowerControl(lslidar_msgs::PowerControl::Request &req,
                                                lslidar_msgs::PowerControl::Response &res) {
        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        int fpga_version = GetCxFpgaVersion(ucwp_data);
        if (fpga_version == 0) {
            ROS_ERROR("LiDAR FPGA error !!!");
            res.result = false;
            return false;
        }

        if (req.power_control == 1) {
            if (fpga_version == 5) {
                ucwp_data[48] == 0xBB;
            } else if (fpga_version == 4) {
                ucwp_data[50] == 0xBB;
            } else if (fpga_version == 3) {
                ucwp_data[8] = 0x02;
                ucwp_data[9] = 0x58;
                ucwp_data[45] = 0x00;
            }
        } else if (req.power_control == 0) {
            if (fpga_version == 5) {
                ucwp_data[48] == 0xAA;
            } else if (fpga_version == 4) {
                ucwp_data[50] == 0xAA;
            } else if (fpga_version == 3) {
                ucwp_data[45] = 0x01;
            }
        }

        sendUcwpPacketTolidar(ucwp_data);
        std::string status = (req.power_control == 1)? "on" : "off";
        ROS_INFO("Set successfully! LiDAR power %s.", status.c_str());
        res.result = true;

        return true;
    }

    bool LslidarCxServices::setRfdRemoval(lslidar_msgs::RfdRemoval::Request &req,
                                              lslidar_msgs::RfdRemoval::Response &res) {
        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        int fpga_version = GetCxFpgaVersion(ucwp_data);
        if (fpga_version <= 3) {
            ROS_WARN("The %d.0 version of LiDAR does not have this function!", fpga_version);
            res.result = false;
            return false;
        }

        if (req.rfd_removal <= 3) {
            if (fpga_version == 5) {
                ucwp_data[49] = static_cast<unsigned char>(req.rfd_removal);
            } else if (fpga_version == 4) {
                ucwp_data[110] = static_cast<unsigned char>(req.rfd_removal);
            }
        }else {
            ROS_ERROR("Parameter error, please check the input parameters.");
            res.result = false;
            return false;
        }

        sendUcwpPacketTolidar(ucwp_data);
        ROS_INFO("Set successfully, Remove rain, fog, and dust, level: %d",req.rfd_removal);
        res.result = true;

        return true;
    }

    bool LslidarCxServices::setTailRemoval(lslidar_msgs::TailRemoval::Request &req,
                                               lslidar_msgs::TailRemoval::Response &res) {
        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        int fpga_version = GetCxFpgaVersion(ucwp_data);
        if (fpga_version <= 3) {
            ROS_WARN("The %d.0 version of LiDAR does not have this function!", fpga_version);
            res.result = false;
            return false;
        }

        if (req.tail_removal <= 10) {
            if (fpga_version == 5) {
                ucwp_data[51] = static_cast<unsigned char>(req.tail_removal);
            } else if (fpga_version == 4) {
                ucwp_data[111] = static_cast<unsigned char>(req.tail_removal);
            }
        }else {
            ROS_ERROR("Parameter error, please check the input parameters.");
            res.result = false;
            return false;
        }
        
        sendUcwpPacketTolidar(ucwp_data);
        ROS_INFO("Set successfully, Remove trail point. level: %d",req.tail_removal);
        res.result = true;

        return true;
    }


    void LslidarCxServices::setTimeModeBytes(unsigned char* ucwp_data, int time_mode) {
        ucwp_data[45] = static_cast<unsigned char>(time_mode);
        // ROS_WARN("****** %d ******", __LINE__);
    }

    bool LslidarCxServices::setTimeMode(lslidar_msgs::TimeMode::Request &req,
                                            lslidar_msgs::TimeMode::Response &res) {

        return LslidarServices::setTimeMode(req, res);
    }



    //////////////////////////////// 905 ////////////////////////////////
    bool LslidarChServices::setIpAndPort(lslidar_msgs::IpAndPort::Request &req,
                                             lslidar_msgs::IpAndPort::Response &res) {

        return LslidarServices::setIpAndPort(req, res);
    }

    bool LslidarChServices::setMotorSpeed(lslidar_msgs::MotorSpeed::Request &req,
                                              lslidar_msgs::MotorSpeed::Response &res) {

        return LslidarServices::setMotorSpeed(req, res);
    }

    bool LslidarChServices::setTimeMode(lslidar_msgs::TimeMode::Request &req,
                                            lslidar_msgs::TimeMode::Response &res) {

        return LslidarServices::setTimeMode(req, res);
    }



    /////////////////////////////// 1550 ///////////////////////////////
    bool LslidarLsServices::setAngleDistortionCorrection(lslidar_msgs::AngleDistortionCorrection::Request &req,
                                                             lslidar_msgs::AngleDistortionCorrection::Response &res) {
        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        if (req.angle_distortion_correction <= 1) {
            ucwp_data[45] = static_cast<unsigned char>(req.angle_distortion_correction);
        } else {
            ROS_ERROR("Parameter error, please check the input parameters.");
            res.result = false;
            return false;
        }

        sendUcwpPacketTolidar(ucwp_data);
        std::string status = (req.angle_distortion_correction == 0)? "off" : "on";
        ROS_INFO("Set successfully! Current LiDAR angle distortion correction %s.", status.c_str());
        ROS_INFO("Please restart the ROS driver.");
        res.result = true;

        return true;
    }

    bool LslidarLsServices::setIpAndPort(lslidar_msgs::IpAndPort::Request &req,
                                             lslidar_msgs::IpAndPort::Response &res) {

        return LslidarServices::setIpAndPort(req, res);
    }

    bool LslidarLsServices::setTimeMode(lslidar_msgs::TimeMode::Request &req,
                                            lslidar_msgs::TimeMode::Response &res) {

        return LslidarServices::setTimeMode(req, res);
    }

    bool LslidarLsServices::setFrameRate(lslidar_msgs::FrameRate::Request &req,
                                             lslidar_msgs::FrameRate::Response &res) {
        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        if (req.frame_rate <= 2) {
            ucwp_data[100] = static_cast<unsigned char>(req.frame_rate);
        } else {
            ROS_ERROR("Parameter error, please check the input parameters.");
            res.result = false;
            return true;
        }

        sendUcwpPacketTolidar(ucwp_data);
        std::string status = (req.frame_rate == 0)? "Standard" : ((req.frame_rate == 1)? "50" : ((req.frame_rate == 2)? "25" : ""));
        ROS_INFO("Set successfully! Current LiDAR %s frame rate.", status.c_str());
        res.result = true;

        return true;
    }

    bool LslidarLsServices::setInvalidData(lslidar_msgs::InvalidData::Request &req,
                                               lslidar_msgs::InvalidData::Response &res) {
        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        if (req.invalid_data <= 1) {
            ucwp_data[87] = static_cast<unsigned char>(req.invalid_data);
        } else {
            ROS_ERROR("Parameter error, please check the input parameters.");
            res.result = false;
            return true;
        }

        sendUcwpPacketTolidar(ucwp_data);
        std::string status = (req.invalid_data == 0)? "send" : "do not send";
        ROS_INFO("Set successfully! Current LiDAR %s invalid data.", status.c_str());
        res.result = true;

        return true;
    }

    bool LslidarLsServices::setStandbyMode(lslidar_msgs::StandbyMode::Request &req,
                                               lslidar_msgs::StandbyMode::Response &res) {
        unsigned char ucwp_data[1206];
        if(!setUcwpData(ucwp_data)) {
            res.result = false;
            return false;
        }

        if (req.standby_mode <= 1) {
            ucwp_data[101] = static_cast<unsigned char>(req.standby_mode);
        } else {
            ROS_ERROR("Parameter error, please check the input parameters.");
            res.result = false;
            return true;
        }

        sendUcwpPacketTolidar(ucwp_data);
        std::string status = (req.standby_mode == 0)? "normal" : "standby";
        ROS_INFO("Set successfully! Current LiDAR is in %s mode.", status.c_str());
        res.result = true;

        return true;
    }
}
