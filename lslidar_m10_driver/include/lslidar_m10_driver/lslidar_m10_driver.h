/*
 * This file is part of lslidar_m10 driver.
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

#ifndef LSLIDAR_M10_DRIVER_H
#define LSLIDAR_M10_DRIVER_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <lslidar_m10_msgs/LslidarM10Packet.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int8.h>
namespace lslidar_m10_driver {

static uint16_t PACKET_SIZE ;

class LslidarM10Driver {
public:

    LslidarM10Driver(ros::NodeHandle& n, ros::NodeHandle& pn);
    ~LslidarM10Driver();

    bool initialize();
    bool polling();
	void difopPoll();

    typedef boost::shared_ptr<LslidarM10Driver> LslidarM10DriverPtr;
    typedef boost::shared_ptr<const LslidarM10Driver> LslidarM10DriverConstPtr;

private:

    bool loadParameters();
    bool createRosIO();
    bool openUDPPort();
    int getPacket(lslidar_m10_msgs::LslidarM10PacketPtr& msg);
	int getDifopPacket(lslidar_m10_msgs::LslidarM10PacketPtr& msg);
    void UDP_order(const std_msgs::Int8 msg);
    void UDP_open(int i);

    // Ethernet relate variables
    std::string device_ip_string;
    std::string difop_ip_string;
    in_addr device_ip;
    int UDP_PORT_NUMBER;
    int UDP_PORT_NUMBER_DIFOP;
    int socket_id;
	int socket_id_difop;
	bool is_start;
    int versions;
    // ROS related variables
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string frame_id;

    ros::Publisher packet_pub;
	ros::Publisher difop_output_;
	ros::Subscriber difop_switch;
	boost::shared_ptr<boost::thread> difop_thread_;
    // Diagnostics updater
    diagnostic_updater::Updater diagnostics;
    boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic;
    double diag_min_freq;
    double diag_max_freq;
};

typedef LslidarM10Driver::LslidarM10DriverPtr LslidarM10DriverPtr;
typedef LslidarM10Driver::LslidarM10DriverConstPtr LslidarM10DriverConstPtr;

} // namespace lslidar_driver

#endif // _LSLIDAR_M10_DRIVER_H_
