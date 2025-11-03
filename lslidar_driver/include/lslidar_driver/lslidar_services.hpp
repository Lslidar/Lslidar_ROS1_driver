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

#ifndef LSLIDAR_SERVICES_HPP
#define LSLIDAR_SERVICES_HPP

#include <atomic>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>  
#include <regex>
#include <cstdio>
#include <string>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "lslidar_msgs/AngleDistortionCorrection.h"
#include "lslidar_msgs/FrameRate.h"
#include "lslidar_msgs/InvalidData.h"
#include "lslidar_msgs/IpAndPort.h"
#include "lslidar_msgs/LslidarPacket.h"
#include "lslidar_msgs/MotorControl.h"
#include "lslidar_msgs/MotorSpeed.h"
#include "lslidar_msgs/PowerControl.h"
#include "lslidar_msgs/RfdRemoval.h"
#include "lslidar_msgs/StandbyMode.h"
#include "lslidar_msgs/TailRemoval.h"
#include "lslidar_msgs/TimeMode.h"

// DIFOP 设备包
// UCWP  配置包

namespace lslidar_driver {

    struct NetConfig {
        std::string lidar_ip;
        std::string destination_ip;
        uint16_t data_port;
        uint16_t dev_port;
    };

    class LslidarServices {
    public:
        LslidarServices() {}

        virtual ~LslidarServices() = default;

        // 获取设备包数据
        int getDifopPacket(lslidar_msgs::LslidarPacketPtr &pkt);

        // 设置配置包包头
        static void setUcwpPacketHeader(unsigned char *ucwp_data);

        // 设置配置包
        bool setUcwpData(unsigned char *ucwp_data);

        // 发送配置包配置雷达
        bool sendUcwpPacketTolidar(unsigned char *ucwp_data) const;

        virtual void setTimeModeBytes(unsigned char* ucwp_data, int time_mode);

        std::string getTimeModeString(int time_mode);

        bool checkAndSetIp(const std::string& ip, unsigned char* ucwp_data, int index, bool is_lidar_ip);

        bool checkAndSetPort(int port, unsigned char* ucwp_data, int index, bool is_data_port);

        bool loadConfigFromYAML();

        // 设置雷达IP 端口
        // 支持 手动传参与配置文件传参
        // 出错保护: 禁止将雷达ip与当前雷达目的ip设置相同，数据端口与当前设备端口设置相同，反之亦然
        virtual bool setIpAndPort(lslidar_msgs::IpAndPort::Request &req,
                                       lslidar_msgs::IpAndPort::Response &res);

        // 设置雷达转速
        virtual bool setMotorSpeed(lslidar_msgs::MotorSpeed::Request &req,
                                       lslidar_msgs::MotorSpeed::Response &res);

        // 设置雷达授时模式
        virtual bool setTimeMode(lslidar_msgs::TimeMode::Request &req,
                                     lslidar_msgs::TimeMode::Response &res);

    protected:
        std::mutex difop_data_mutex_;
        std::atomic<bool> difop_valid_{false};
        unsigned char difop_data[1206];
        std::string lslidar_ip_;

        NetConfig LiDAR;
    };


    class LslidarCxServices : public LslidarServices {
    public:
        LslidarCxServices() {}

        // 机械式雷达设置 IP 端口
        bool setIpAndPort(lslidar_msgs::IpAndPort::Request &req,
                              lslidar_msgs::IpAndPort::Response &res) override;

        // 机械式雷达设置转速
        bool setMotorSpeed(lslidar_msgs::MotorSpeed::Request &req,
                               lslidar_msgs::MotorSpeed::Response &res) override;
        
        // 机械式雷达控制电机启停
        bool setMotorControl(lslidar_msgs::MotorControl::Request &req,
                                 lslidar_msgs::MotorControl::Response &res);
        
        // 机械式雷达控制雷达上下电
        bool setPowerControl(lslidar_msgs::PowerControl::Request &req,
                                 lslidar_msgs::PowerControl::Response &res);
        
        // 机械式雷达去雨雾尘
        bool setRfdRemoval(lslidar_msgs::RfdRemoval::Request &req,
                               lslidar_msgs::RfdRemoval::Response &res);
        
        // 机械式雷达设置去除拖尾
        bool setTailRemoval(lslidar_msgs::TailRemoval::Request &req,
                                lslidar_msgs::TailRemoval::Response &res);

        // 机械式雷达设置授时模式
        bool setTimeMode(lslidar_msgs::TimeMode::Request &req,
                             lslidar_msgs::TimeMode::Response &res) override;
        
        void setTimeModeBytes(unsigned char* ucwp_data, int time_mode) override;

        int GetCxFpgaVersion(unsigned char* ucwp_data);
    };


    /////////////////////// 905 ///////////////////////
    class LslidarChServices : public LslidarServices {
    public:
        LslidarChServices() {}

        // 905雷达设置 IP 端口
        bool setIpAndPort(lslidar_msgs::IpAndPort::Request &req,
                              lslidar_msgs::IpAndPort::Response &res) override;

        // 905雷达设置转速
        bool setMotorSpeed(lslidar_msgs::MotorSpeed::Request &req,
                               lslidar_msgs::MotorSpeed::Response &res) override;

        bool setTimeMode(lslidar_msgs::TimeMode::Request &req,
                             lslidar_msgs::TimeMode::Response &res) override;
    };



    ////////////////////// 1550 //////////////////////
    class LslidarLsServices : public LslidarServices {
    public:
        LslidarLsServices() {}

        // 1550雷达设置角度畸变矫正
        bool setAngleDistortionCorrection(lslidar_msgs::AngleDistortionCorrection::Request &req,
                                              lslidar_msgs::AngleDistortionCorrection::Response &res);

        // 1550雷达设置 IP 端口
        bool setIpAndPort(lslidar_msgs::IpAndPort::Request &req,
                              lslidar_msgs::IpAndPort::Response &res) override;

        // 1550雷达设置转速  *** 1550没有该功能 ***
        // bool setMotorSpeed(lslidar_msgs::MotorSpeed::Request &req,
        //                        lslidar_msgs::MotorSpeed::Response &res) override;
        
        bool setTimeMode(lslidar_msgs::TimeMode::Request &req,
                             lslidar_msgs::TimeMode::Response &res) override;

        // 1550雷达设置帧率
        bool setFrameRate(lslidar_msgs::FrameRate::Request &req,
                              lslidar_msgs::FrameRate::Response &res);

        // 1550雷达设置是否发布无效数据
        bool setInvalidData(lslidar_msgs::InvalidData::Request &req,
                                lslidar_msgs::InvalidData::Response &res);

        // 1550雷达设置待机模式
        bool setStandbyMode(lslidar_msgs::StandbyMode::Request &req,
                                lslidar_msgs::StandbyMode::Response &res);
    };
}

#endif // LSLIDAR_SERVICES_HPP
