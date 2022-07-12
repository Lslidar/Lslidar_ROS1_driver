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

#ifndef LSLIDAR_M10_DECODER_H
#define LSLIDAR_M10_DECODER_H

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

#include <cmath>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include "time.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <lslidar_m10_msgs/LslidarM10Packet.h>
#include <lslidar_m10_msgs/LslidarM10Difop.h>
#include <lslidar_m10_msgs/LslidarM10Point.h>
#include <lslidar_m10_msgs/LslidarM10Scan.h>
#include <lslidar_m10_msgs/LslidarM10Sweep.h>
#include <std_msgs/Byte.h>

namespace lslidar_m10_decoder {

/** Special Defines for VLP16 support **/
static const int     SCANS_PER_FIRING  = 1;
static const int     PACKET_SIZE       = 2000;

class LslidarM10Decoder {
public:

    LslidarM10Decoder(ros::NodeHandle& n, ros::NodeHandle& pn);
    LslidarM10Decoder(const LslidarM10Decoder&) = delete;
    LslidarM10Decoder operator=(const LslidarM10Decoder&) = delete;
    ~LslidarM10Decoder() {return;}

    bool initialize();

    typedef boost::shared_ptr<LslidarM10Decoder> LslidarM10DecoderPtr;
    typedef boost::shared_ptr<const LslidarM10Decoder> LslidarM10DecoderConstPtr;

private:
	typedef struct {
		double degree;
		double range;
		double intensity;
	} ScanPoint;

    union TwoBytes {
        uint16_t distance;
        uint8_t  bytes[2];
    };

    // Intialization sequence
    bool loadParameters();
    bool createRosIO();

    // Callback function for a single lslidar packet.
    void packetCallback(const lslidar_m10_msgs::LslidarM10PacketConstPtr& msg);
	void processDifop(const lslidar_m10_msgs::LslidarM10Packet::ConstPtr& difop_msg);

    // Publish scan Data
    void publishScan();

    // Check if a point is in the required range.
    bool isPointInRange(const double& distance) {
        return (distance >= min_range && distance <= max_range);
    }

    double rawAzimuthToDouble(const uint16_t& raw_azimuth) {
        return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
    }

    // configuration degree base
    int versions;
	size_t scan_size;
	int idx;
    int point_num;
    int count_num;
    // Configuration parameters
    double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
	double angle_able_min;
    double angle_able_max;
    double last_degree;
	double degree;

    int lidar_frequency;
    int package_points;
    int data_bits_start;
    int degree_bits_start;
    int rpm_bits_start;
	std::vector<ScanPoint> scan_points_;
	
    // ROS related parameters
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    std::string child_frame_id;
	std::string scan_topic_;
	lslidar_m10_msgs::LslidarM10DifopPtr Difop_data;

    ros::Subscriber packet_sub;
	ros::Subscriber difop_sub_;

    ros::Publisher scan_pub;
	ros::Publisher device_pub;
};

typedef LslidarM10Decoder::LslidarM10DecoderPtr LslidarM10DecoderPtr;
typedef LslidarM10Decoder::LslidarM10DecoderConstPtr LslidarM10DecoderConstPtr;

} // end namespace lslidar_m10_decoder

#endif
