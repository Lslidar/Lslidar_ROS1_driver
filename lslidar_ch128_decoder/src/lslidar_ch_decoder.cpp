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

#include <lslidar_ch_decoder/lslidar_ch_decoder.h>
#include <std_msgs/Int8.h>

using namespace std;

namespace lslidar_ch_decoder {
    LslidarChDecoder::LslidarChDecoder(
            ros::NodeHandle &n, ros::NodeHandle &pn) :
            nh(n),
            pnh(pn),
            publish_point_cloud(true),
            is_first_sweep(true),
            last_azimuth(0.0),
            sweep_start_time(0.0),
            sweep_data(new lslidar_ch128_msgs::LslidarChScan()) {
        packet_timeStamp.sec = 0;
        packet_timeStamp.nsec = 0;

        return;
    }

    bool LslidarChDecoder::loadParameters() {
        pnh.param<double>("min_range", min_range, 0.5);
        pnh.param<double>("max_range", max_range, 100.0);
        pnh.param<int>("frequency", frequency, 10);
        pnh.param<string>("lslidar_point_cloud", lslidar_point_cloud, "lslidar_pointcloud_ch128");
        pnh.param<bool>("publish_point_cloud", publish_point_cloud, true);
        pnh.param<bool>("publish_scan", publish_scan, false);
        pnh.param<int>("channel_num", channel_num, 8);
        pnh.param<bool>("apollo_interface", apollo_interface, false);
        //pnh.param<string>("fixed_frame_id", fixed_frame_id, "map");
        pnh.param<string>("frame_id", frame_id, "lslidar");
        pnh.param<bool>("use_gps_ts", use_gps_ts, false);
        ROS_INFO("Using GPS timestamp or not %d", use_gps_ts);
        if (channel_num < 0) {
            channel_num = 0;
            ROS_WARN("channel_num outside of the index, select channel 0 instead!");
        } else if (channel_num > 127) {
            channel_num = 127;
            ROS_WARN("channel_num outside of the index, select channel 15 instead!");
        }
        ROS_INFO("select channel num: %d", channel_num);
        if (apollo_interface)
            ROS_WARN("This is apollo interface mode");
        switch (frequency) {
            case 5:
                horizontal_angle_resolution = DEG2RAD(0.09);
                break;
            case 15:
                horizontal_angle_resolution = DEG2RAD(0.36);
                break;
            default:
                horizontal_angle_resolution = DEG2RAD(0.18);
        }

        return true;
    }

    bool LslidarChDecoder::createRosIO() {
        packet_sub = nh.subscribe<lslidar_ch128_msgs::LslidarChPacket>(
                "lslidar_packet_ch128", 100, &LslidarChDecoder::packetCallback, this);
        sweep_pub = nh.advertise<lslidar_ch128_msgs::LslidarChSweep>(
                "lslidar_sweep", 10);
        point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
                lslidar_point_cloud, 10);
        laserscan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
        return true;
    }

    bool LslidarChDecoder::initialize() {
        if (!loadParameters()) {
            ROS_ERROR("Cannot load all required parameters...");
            return false;
        }

        if (!createRosIO()) {
            ROS_ERROR("Cannot create ROS I/O...");
            return false;
        }

        return true;
    }

    void LslidarChDecoder::publishPointCloud() {

        pcl::PointCloud<VPoint>::Ptr point_cloud(new pcl::PointCloud<VPoint>);
        point_cloud->header.frame_id = frame_id;
        point_cloud->height = 1;

        // The first and last point in each scan is ignored, which
        // seems to be corrupted based on the received data.
        // TODO: The two end points should be removed directly
        //    in the scans.
        double timestamp = ros::Time::now().toSec();

        size_t j;
        VPoint point;
        if (sweep_data->points.size() > 0) {
            if (use_gps_ts) {
                point_cloud->header.stamp = pcl_conversions::toPCL(packet_timeStamp);
            } else {
                point_cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
            }

            for (j = 1; j < sweep_data->points.size() - 1; ++j) {
                point.time = sweep_data->points[j].time;
                point.x = sweep_data->points[j].x;
                point.y = sweep_data->points[j].y;
                point.z = sweep_data->points[j].z;
                point.intensity = sweep_data->points[j].intensity;
                point.ring = sweep_data->points[j].line;

                point_cloud->points.push_back(point);
                ++point_cloud->width;
            }

            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            point_cloud_pub.publish(pc_msg);
        }
        return;
    }

    void LslidarChDecoder::publishLaserScan() {

        sensor_msgs::LaserScan::Ptr scan_msg(new sensor_msgs::LaserScan);

        scan_msg->header.frame_id = frame_id;
        scan_msg->angle_min = DEG2RAD(15);
        scan_msg->angle_max = DEG2RAD(165);
        scan_msg->range_min = min_range;
        scan_msg->range_max = max_range;
        scan_msg->angle_increment = horizontal_angle_resolution;
        uint point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());

        // The first and last point in each scan is ignored, which
        // seems to be corrupted based on the received data.
        // TODO: The two end points should be removed directly
        //    in the scans.

        if (use_gps_ts) {
            scan_msg->header.stamp = packet_timeStamp;
        } else {
            scan_msg->header.stamp = ros::Time::now();
        }
        if (sweep_data->points.size() > 0) {
            for (size_t j = 0; j < sweep_data->points.size() - 1; ++j) {
                if (channel_num == sweep_data->points[j].line) {
                    float horizontal_angle = sweep_data->points[j].azimuth;
                    uint point_idx = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                    point_idx = (point_idx <= point_size) ? point_idx : (point_idx % point_size);
                    scan_msg->ranges[point_idx] = sweep_data->points[j].distance;
                    scan_msg->intensities[point_idx] = sweep_data->points[j].intensity;
                }
            }
            laserscan_pub.publish(scan_msg);
        }

        return;
    }


    void LslidarChDecoder::decodePacket(const RawPacket *packet) {

        // Compute the values for each firing
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {
            //firings[point_idx].vertical_angle=verticalLineToAngle(packet->points[point_idx].vertical_line);
            firings[point_idx].vertical_line = packet->points[point_idx].vertical_line;
            TwoBytes point_amuzith;
            point_amuzith.bytes[0] = packet->points[point_idx].azimuth_2;
            point_amuzith.bytes[1] = packet->points[point_idx].azimuth_1;
            firings[point_idx].azimuth = static_cast<double>(point_amuzith.value) * 0.01 * DEG_TO_RAD;
            ThreeBytes point_distance;
            point_distance.bytes[0] = packet->points[point_idx].distance_3;
            point_distance.bytes[1] = packet->points[point_idx].distance_2;
            point_distance.bytes[2] = packet->points[point_idx].distance_1;
            firings[point_idx].distance = static_cast<double>(point_distance.value) * DISTANCE_RESOLUTION;
            firings[point_idx].intensity = packet->points[point_idx].intensity;
        }
        return;
    }


    int LslidarChDecoder::checkPacketValidity(const RawPacket *packet) {

        //ROS_WARN("packet factory is %2x  %2x", packet->factory[0],packet->factory[1]);
        for (size_t blk_idx = 0; blk_idx < POINTS_PER_PACKET; blk_idx++) {
            if ((packet->points[blk_idx].vertical_line == 0xff) && (packet->points[blk_idx].azimuth_1 == 0xaa) &&
                (packet->points[blk_idx].azimuth_2 == 0xbb)) {
                //&&(packet->points[blk_idx].distance_1 ==0xcc)
                //&&(packet->points[blk_idx].distance_2 ==0xdd)&&(packet->points[blk_idx].intensity == 0xee)) {
                // ROS_WARN("LSch packet: block %lu vertical_line is %x %x ",blk_idx, packet->points[blk_idx].vertical_line,packet->points[blk_idx].intensity);
                return true;
            }
        }
        return false;

    }

    void LslidarChDecoder::packetCallback(
            const lslidar_ch128_msgs::LslidarChPacketConstPtr &msg) {

        // Convert the msg to the raw packet type.
        const RawPacket *raw_packet = (const RawPacket *) (&(msg->data[0]));
        packet_timeStamp = msg->header.stamp;
        packet_end_time = packet_timeStamp.toSec();

        // Check if the packet is valid and find the header of frame
        bool packetType = checkPacketValidity(raw_packet);

        // Decode the packet
        decodePacket(raw_packet);
        double x = 0.0, y = 0.0, z = 0.0;
        double z_sin_altitude = 0.0;
        double z_cos_altitude = 0.0;
        double cos_azimuth_half = 0.0;
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {

            // Check if the point is valid.
            if (!isPointInRange(firings[point_idx].distance)) continue;

            // Convert the point to xyz coordinate
            if (firings[point_idx].vertical_line / 4 % 2 == 0) {
                cos_azimuth_half = sin((firings[point_idx].azimuth - 15 * DEG_TO_RAD) * 0.5);
                // cos_azimuth_half=cos((firings[point_idx].azimuth+15*DEG_TO_RAD)*0.5);
            } else {
                // cos_azimuth_half=sin((firings[point_idx].azimuth-15*DEG_TO_RAD)*0.5);
                cos_azimuth_half = cos((firings[point_idx].azimuth + 15 * DEG_TO_RAD) * 0.5);
            }

            if (abs(msg->prism_angle[0]) < 1e-6 && abs(msg->prism_angle[1]) < 1e-6 && abs(msg->prism_angle[2]) < 1e-6 &&
                abs(msg->prism_angle[3]) < 1e-6){
                z_sin_altitude = sin_scan_laser_altitude[firings[point_idx].vertical_line / 4] +
                                 2 * cos_azimuth_half * sin_scan_mirror_altitude[firings[point_idx].vertical_line % 4];
            }else{
                z_sin_altitude = sin_scan_laser_altitude[firings[point_idx].vertical_line / 4] +
                                 2 * cos_azimuth_half * std::sin(msg->prism_angle[firings[point_idx].vertical_line % 4] * M_PI /180);

            }

            z_cos_altitude = sqrt(1 - z_sin_altitude * z_sin_altitude);
            x = firings[point_idx].distance * z_cos_altitude * cos(firings[point_idx].azimuth);
            y = firings[point_idx].distance * z_cos_altitude * sin(firings[point_idx].azimuth);
            z = firings[point_idx].distance * z_sin_altitude;
            double x_coord = x;
            double y_coord = y;
            double z_coord = z;

            // Compute the time of the point
            double point_time = packet_end_time - 1.5625 * (POINTS_PER_PACKET - point_idx - 1) * 1e-6;

            sweep_data->points.push_back(lslidar_ch128_msgs::LslidarChPoint());
            lslidar_ch128_msgs::LslidarChPoint &new_point =        // new_point 为push_back最后一个的引用
                    sweep_data->points[sweep_data->points.size() - 1];
            // Pack the data into point msg
            new_point.time = point_time;
            new_point.x = x_coord;
            new_point.y = y_coord;
            new_point.z = z_coord;
            new_point.vertical_angle = verticalLineToAngle(firings[point_idx].vertical_line);
            new_point.azimuth = firings[point_idx].azimuth;
            new_point.distance = firings[point_idx].distance;
            new_point.intensity = firings[point_idx].intensity;
            new_point.line = firings[point_idx].vertical_line;
        }
        if (packetType) {
            if (publish_point_cloud)publishPointCloud();
            if (publish_scan)publishLaserScan();
            sweep_data = lslidar_ch128_msgs::LslidarChScanPtr(
                    new lslidar_ch128_msgs::LslidarChScan());
        }

        return;
    }

} // end namespace lslidar_ch_decoder

