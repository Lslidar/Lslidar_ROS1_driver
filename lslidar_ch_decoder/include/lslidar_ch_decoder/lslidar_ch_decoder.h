/*
 * This file is part of lslidar_ch driver.
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

#ifndef LSLIDAR_Ch_DECODER_H
#define LSLIDAR_Ch_DECODER_H

#define DEG_TO_RAD 0.017453292
#define RAD_TO_DEG 57.29577951

#include <cmath>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <lslidar_ch_msgs/LslidarChPacket.h>
#include <lslidar_ch_msgs/LslidarChPoint.h>
#include <lslidar_ch_msgs/LslidarChScan.h>
#include <lslidar_ch_msgs/LslidarChSweep.h>
#include <lslidar_ch_msgs/LslidarChLayer.h>


namespace lslidar_ch_decoder {

// Raw lslidar packet constants and structures.
static const int SIZE_BLOCK      = 100;
static const int RAW_SCAN_SIZE   = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE =
        (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

// valid packets with readings up to 200.0.
static const double DISTANCE_MAX        = 200.0;        /**< meters */
static const double DISTANCE_RESOLUTION = 0.0000390625; /**< meters */
static const double DISTANCE_MAX_UNITS  =
        (DISTANCE_MAX / DISTANCE_RESOLUTION + 1.0);

/** @todo make this work for both big and little-endian machines */
//static const uint8_t UPPER_BANK_ONE = 0xff;   //flag 0
//static const uint16_t UPPER_BANK_TWO =0xbbaa;    //flag 1
//static const uint16_t UPPER_BANK_THREE=0xddcc;    //flag 2
//static const uint8_t UPPER_BANK_FOUR = 0xee;  //flag 3

/** Special Defines for LSCh support **/
static const int PACKET_SIZE        = 1206;    
static const int POINTS_PER_PACKET  = 171;

// Pre-compute the sine and cosine for the altitude angles.
static const double scan_laser_altitude[32] = {
        -0.29670597283903605,-0.2792526803190927,
        -0.2617993877991494,-0.24434609527920614,
        -0.22689280275926285,-0.20943951023931956,
        -0.19198621771937624,-0.17453292519943295,
        -0.15707963267948966,-0.13962634015954636,
        -0.12217304763960307,-0.10471975511965978,
        -0.08726646259971647,-0.06981317007977318,
        -0.05235987755982989,-0.03490658503988659,
        -0.017453292519943295,0.0,
         0.017453292519943295,0.03490658503988659,
         0.05235987755982989,0.06981317007977318,
         0.08726646259971647,0.10471975511965978,
         0.12217304763960307,0.13962634015954636,
         0.15707963267948966,0.17453292519943295,
         0.19198621771937624,0.20943951023931956,
         0.22689280275926285,0.24434609527920614,
};

static const double sin_scan_laser_altitude[32] = {
    std::sin(scan_laser_altitude[0]), std::sin(scan_laser_altitude[1]),
    std::sin(scan_laser_altitude[2]), std::sin(scan_laser_altitude[3]),
    std::sin(scan_laser_altitude[4]), std::sin(scan_laser_altitude[5]),
    std::sin(scan_laser_altitude[6]), std::sin(scan_laser_altitude[7]),
    std::sin(scan_laser_altitude[8]), std::sin(scan_laser_altitude[9]),
    std::sin(scan_laser_altitude[10]), std::sin(scan_laser_altitude[11]),
    std::sin(scan_laser_altitude[12]), std::sin(scan_laser_altitude[13]),
    std::sin(scan_laser_altitude[14]), std::sin(scan_laser_altitude[15]),
    std::sin(scan_laser_altitude[16]), std::sin(scan_laser_altitude[17]),
    std::sin(scan_laser_altitude[18]), std::sin(scan_laser_altitude[19]),
    std::sin(scan_laser_altitude[20]), std::sin(scan_laser_altitude[21]),
    std::sin(scan_laser_altitude[22]), std::sin(scan_laser_altitude[23]),
    std::sin(scan_laser_altitude[24]), std::sin(scan_laser_altitude[25]),
    std::sin(scan_laser_altitude[26]), std::sin(scan_laser_altitude[27]),
    std::sin(scan_laser_altitude[28]), std::sin(scan_laser_altitude[29]),
    std::sin(scan_laser_altitude[30]), std::sin(scan_laser_altitude[31]),
};

static const double scan_mirror_altitude[4] = {
        0.0,0.004363323129985824,
        0.008726646259971648,0.013089969389957472,
};

static const double sin_scan_mirror_altitude[4] = {
    std::sin(scan_mirror_altitude[0]), std::sin(scan_mirror_altitude[1]),
    std::sin(scan_mirror_altitude[2]), std::sin(scan_mirror_altitude[3]),
};

typedef struct{
    double distance;
    double intensity;
}point_struct;


struct PointXYZITM {
  PCL_ADD_POINT4D
  uint8_t intensity;
  uint8_t lines;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
// enforce SSE padding for correct memory alignment


class LslidarChDecoder {
public:
    LslidarChDecoder(ros::NodeHandle& n, ros::NodeHandle& pn);
    LslidarChDecoder(const LslidarChDecoder&) = delete;
    LslidarChDecoder operator=(const LslidarChDecoder&) = delete;
    ~LslidarChDecoder() {return;}

    bool initialize();

    typedef boost::shared_ptr<LslidarChDecoder> LslidarChDecoderPtr;
    typedef boost::shared_ptr<const LslidarChDecoder> LslidarChDecoderConstPtr;

private:

    union TwoBytes {
        uint16_t value;
        uint8_t  bytes[2];
    };
    
    union ThreeBytes {
        uint32_t value;
        uint8_t  bytes[3];
    };

    struct Point {
        uint8_t vertical_line;        //0-127 
        uint8_t azimuth_1;
        uint8_t azimuth_2;      ///< 1500-16500, divide by 100 to get degrees
        uint8_t distance_1;
        uint8_t distance_2;
        uint8_t distance_3;
        uint8_t intensity;
    };
    
    struct RawPacket {
        Point points[POINTS_PER_PACKET];
        uint32_t time_stamp;
        uint8_t factory[2];
    };

    struct Firing {
        //double vertical_angle;
        int vertical_line;
        double azimuth;
        double distance;
        int intensity;
    };

    // Intialization sequence
    bool loadParameters();
    bool createRosIO();


    // Callback function for a single lslidar packet.
    int checkPacketValidity(const RawPacket* packet);
    void decodePacket(const RawPacket* packet);
    void layerCallback(const std_msgs::Int8Ptr& msg);
    void packetCallback(const lslidar_ch_msgs::LslidarChPacketConstPtr& msg);
    // Publish data
    void publishPointCloud();
    void publishChannel();
    // Check if a point is in the required range.
    bool isPointInRange(const double& distance) {
        return (distance >= min_range && distance <= max_range);
    }

    double rawAzimuthToDouble(const uint16_t& raw_azimuth) {
        // According to the user manual,
        return static_cast<double>(raw_azimuth) / 100.0 * DEG_TO_RAD;
		//return static_cast<double>(raw_azimuth) / 100.0;
    }
     double verticalLineToAngle(const uint16_t& vertical_line) {
        // According to the user manual,
       // return static_cast<double>(vertical_line)*0.25-17;
                return (static_cast<double>(vertical_line)*0.167-13) * DEG_TO_RAD;
               // return (static_cast<double>(vertical_line)*0.25-17) * DEG_TO_RAD;
    }
    
    // configuration degree base
    int point_num;
    double angle_base;
    
    // Configuration parameters
    double min_range;
    double max_range;
    double frequency;
    bool publish_point_cloud;
    bool publish_channel;
    bool use_gps_ts;
    bool apollo_interface;
    bool time_synchronization_;
    ros::Time packet_timeStamp;
    ros::Time packet_end_timeStamp;

    //double cos_azimuth_table[6300];
    //double sin_azimuth_table[6300];

    bool is_first_sweep;
    double last_azimuth;
    double sweep_start_time;
    double packet_start_time;
    uint64_t time_last;
    int channel_num;

    int layer_num;
    Firing firings[POINTS_PER_PACKET];

    // ROS related parameters
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    //std::string fixed_frame_id;
    std::string frame_id;
    std::string lslidar_point_cloud;
   
    sensor_msgs::PointCloud2 point_cloud_data;
    lslidar_ch_msgs::LslidarChScanPtr sweep_data;

    ros::Subscriber packet_sub;
    ros::Subscriber layer_sub;
    ros::Publisher sweep_pub;
    ros::Publisher point_cloud_pub;
    ros::Publisher channel_pub;
};

typedef LslidarChDecoder::LslidarChDecoderPtr LslidarChDecoderPtr;
typedef LslidarChDecoder::LslidarChDecoderConstPtr LslidarChDecoderConstPtr;
typedef PointXYZITM VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

} // end namespace lslidar_ch_decoder


POINT_CLOUD_REGISTER_POINT_STRUCT(lslidar_ch_decoder::PointXYZITM,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (uint8_t, intensity, intensity)
                                  (uint8_t, lines, lines)
                                  (double, timestamp, timestamp)
                                  )
#endif
