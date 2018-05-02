/*
 * This file is part of lslidar_c16 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LSLIDAR_C16_DRIVER_H
#define LSLIDAR_C16_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <lslidar_c16_msgs/LslidarC16Packet.h>
#include <lslidar_c16_msgs/LslidarC16ScanUnified.h>

namespace lslidar_c16_driver {

//static uint16_t UDP_PORT_NUMBER = 8080;
static uint16_t PACKET_SIZE = 1206;

class LslidarC16Driver {
public:

    LslidarC16Driver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarC16Driver();

    bool initialize();
    bool polling();

    typedef boost::shared_ptr<LslidarC16Driver> LslidarC16DriverPtr;
    typedef boost::shared_ptr<const LslidarC16Driver> LslidarC16DriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_c16_msgs::LslidarC16PacketPtr& msg);

    // Ethernet relate variables
    std::string device_ip_string;
    in_addr device_ip;
    int UDP_PORT_NUMBER;
    int socket_id;

    // ROS related variables
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string frame_id;
    ros::Publisher packet_pub;    

    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;
};

typedef LslidarC16Driver::LslidarC16DriverPtr LslidarC16DriverPtr;
typedef LslidarC16Driver::LslidarC16DriverConstPtr LslidarC16DriverConstPtr;

} // namespace lslidar_driver

#endif // _LSLIDAR_C16_DRIVER_H_
