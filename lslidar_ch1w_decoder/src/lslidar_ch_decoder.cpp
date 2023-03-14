
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
            current_packet_time(0),
            last_packet_time(0),
            sweep_data(new lslidar_ch1w_msgs::LslidarChScan()) {
        return;
    }

    bool LslidarChDecoder::loadParameters() {
        pnh.param<double>("min_range", min_range, 0.5);
        pnh.param<double>("max_range", max_range, 100.0);
        pnh.param<double>("angle_disable_min", angle_disable_min, 0.0);
        pnh.param<double>("angle_disable_max", angle_disable_max, 0.0);
        pnh.param<double>("pulse_repetition_rate", pulse_repetition_rate, 60.0);
        pnh.param<int>("frequency", frequency, 10);
        pnh.param<string>("pointcloud_topic", pointcloud_topic, "lslidar_point_cloud");
        pnh.param<string>("scan_channel_topic", scan_channel_topic, "scan_channel");
        pnh.param<bool>("publish_point_cloud", publish_point_cloud, true);
        pnh.param<bool>("publish_laserscan", publish_laserscan, false);
        pnh.param<bool>("pcl_type", pcl_type, false);
        //pnh.param<string>("fixed_frame_id", fixed_frame_id, "map");
        pnh.param<string>("frame_id", frame_id, "lslidar");
        while (angle_disable_min < 0) angle_disable_min += 180;
        while (angle_disable_min > 180) angle_disable_min -= 180;
        while (angle_disable_max < 0) angle_disable_max += 180;
        while (angle_disable_max > 180) angle_disable_max -= 180;
        if (angle_disable_min > angle_disable_max) angle_disable_max += 180;

        angle_disable_min = angle_disable_min * DEG_TO_RAD;
        angle_disable_max = angle_disable_max * DEG_TO_RAD;
        switch (frequency) {
            case 5:
                horizontal_angle_resolution = DEG2RAD(0.06);
                break;
            case 20:
                horizontal_angle_resolution = DEG2RAD(0.24);
                break;
            default:
                horizontal_angle_resolution = DEG2RAD(0.12);
        }

        return true;
    }

    bool LslidarChDecoder::createRosIO() {
        packet_sub = nh.subscribe<lslidar_ch1w_msgs::LslidarChPacket>(
                "lslidar_packet_1w", 100, &LslidarChDecoder::packetCallback, this);
        point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
                pointcloud_topic, 10);
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
        point_time_diff = 1.0 / (pulse_repetition_rate * 1000 * 32);  // s
        return true;
    }

    void LslidarChDecoder::publishPointCloud() {
        if (pcl_type) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;
            pcl::PointXYZI point;
            if (sweep_data->points.size() > 0) {

                point_cloud->header.stamp = point_cloud_timestamp;

                for (size_t j = 0; j < sweep_data->points.size(); ++j) {
                    if ((sweep_data->points[j].azimuth > angle_disable_min) &&
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
                sensor_msgs::PointCloud2 pc_msg;
                pcl::toROSMsg(*point_cloud, pc_msg);
                point_cloud_pub.publish(pc_msg);
            }
        } else {

            pcl::PointCloud<VPoint>::Ptr point_cloud(new pcl::PointCloud<VPoint>);
            point_cloud->header.frame_id = frame_id;
            point_cloud->height = 1;
            //pcl::PointXYZI point;
            VPoint point;
//            ROS_INFO("sweep_data->points.size(): %d", sweep_data->points.size());

            if (sweep_data->points.size() > 0) {
                point_cloud->header.stamp = point_cloud_timestamp;
                for (size_t j = 0; j < sweep_data->points.size(); ++j) {
                    if ((sweep_data->points[j].azimuth > angle_disable_min) &&
                        (sweep_data->points[j].azimuth < angle_disable_max)) {
                        continue;
                    }
                    point.time = sweep_data->points[j].time;
                    point.x = sweep_data->points[j].x;
                    point.y = sweep_data->points[j].y;
                    point.z = sweep_data->points[j].z;
                    point.intensity = sweep_data->points[j].intensity;
                    point.ring = sweep_data->points[j].line;
                    point_cloud->points.push_back(point);
                    ++point_cloud->width;
                    current_point_time = point.time;
                    last_point_time = current_point_time;
                }
                sensor_msgs::PointCloud2 pc_msg;
                pcl::toROSMsg(*point_cloud, pc_msg);
                point_cloud_pub.publish(pc_msg);
            }
//            ROS_INFO("point_cloud->width %d", point_cloud->width);
        }
        return;
    }

    void LslidarChDecoder::publishLaserScan() {
        sensor_msgs::LaserScan::Ptr scan_msg(new sensor_msgs::LaserScan);
        scan_msg->header.frame_id = frame_id;
        scan_msg->angle_min = DEG2RAD(0);
        scan_msg->angle_max = DEG2RAD(180);
//        scan_msg->angle_min = angle_disable_min;
//        scan_msg->angle_max = angle_disable_max;
        scan_msg->range_min = min_range;
        scan_msg->range_max = max_range;
//        scan_msg->angle_increment = horizontal_angle_resolution;
        if (sweep_data->points.size() > 0) {
            scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / sweep_data->points.size();
        }
        uint point_size = ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment) * 2;
        scan_msg->ranges.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        scan_msg->intensities.assign(point_size, std::numeric_limits<float>::quiet_NaN());
        scan_msg->header.stamp = pcl_conversions::fromPCL(point_cloud_timestamp);

        int points_num = 0;
        if (sweep_data->points.size() > 0) {
            for (size_t j = 0; j < sweep_data->points.size() - 1; ++j) {
                float horizontal_angle = sweep_data->points[j].azimuth;
                if ((sweep_data->points[j].azimuth > angle_disable_min) &&
                    (sweep_data->points[j].azimuth < angle_disable_max)) {
                    continue;
                }
                uint point_idx = (int) ((horizontal_angle - scan_msg->angle_min) / scan_msg->angle_increment);
                point_idx = (point_idx <= point_size) ? point_idx : (point_idx % point_size);
                scan_msg->ranges[point_idx] = sweep_data->points[j].distance;
                scan_msg->intensities[point_idx] = sweep_data->points[j].intensity;
                points_num++;
            }

            laserscan_pub.publish(scan_msg);
//            ROS_INFO("points_num %d", points_num);
        }
        return;
    }

    void LslidarChDecoder::decodePacket(const RawPacket *packet) {

        // Compute the values for each firing
        for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {

            firings[point_idx].vertical_line = packet->points[point_idx].vertical_line;
            TwoBytes point_amuzith{};
            point_amuzith.bytes[0] = packet->points[point_idx].azimuth_2;
            point_amuzith.bytes[1] = packet->points[point_idx].azimuth_1;
            firings[point_idx].azimuth = static_cast<double>(point_amuzith.value) * 0.01;
            ThreeBytes point_distance{};
            point_distance.bytes[0] = packet->points[point_idx].distance1_3;
            point_distance.bytes[1] = packet->points[point_idx].distance1_2;
            point_distance.bytes[2] = packet->points[point_idx].distance1_1;
            firings[point_idx].distance = static_cast<double>(point_distance.value) * DISTANCE_RESOLUTION;
            firings[point_idx].intensity = packet->points[point_idx].intensity1;
        }
        return;
    }


    int LslidarChDecoder::convertCoordinate(const Firing lidardata) {
        if (!isPointInRange(lidardata.distance)) {
            return -1;
        }
        int line_num = lidardata.vertical_line;
        double x = 0.0, y = 0.0, z = 0.0, add_distance;
        if (line_num / 4 % 2 == 0) {
            add_distance = 0.017;
        } else {
            add_distance = -0.017;
        }

        x = lidardata.distance * cos(lidardata.azimuth * M_PI / 180) + add_distance;
        y = lidardata.distance * sin(lidardata.azimuth * M_PI / 180);
        z = 0.0;

        if (((lidardata.azimuth * M_PI / 180 > angle_disable_min) &&
             (lidardata.azimuth * M_PI / 180 < angle_disable_max)) || lidardata.azimuth > 180) {
            return 0;
        }

        sweep_data->points.push_back(lslidar_ch1w_msgs::LslidarChPoint());
        lslidar_ch1w_msgs::LslidarChPoint &new_point =        // new_point 为push_back最后一个的引用
                sweep_data->points[sweep_data->points.size() - 1];
        // Pack the data into point msg
        new_point.time = lidardata.time;
        new_point.x = x;
        new_point.y = y;
        new_point.z = z;
        new_point.azimuth = lidardata.azimuth * M_PI / 180;
//        ROS_INFO("lidardata.azimuth %f",lidardata.azimuth);
//        ROS_INFO("new_point.azimuth %f",new_point.azimuth);
//        ROS_INFO("sweep_data .azimuth %f",sweep_data->points[sweep_data->points.size() - 1].azimuth);
        new_point.distance = lidardata.distance;
        new_point.intensity = lidardata.intensity;
        new_point.line = lidardata.vertical_line;
        return 0;
    }


    void LslidarChDecoder::packetCallback(
            const lslidar_ch1w_msgs::LslidarChPacketConstPtr &msg) {

        struct Firing lidardata;

        packet_end_time = msg->stamp.toNSec();
        current_packet_time = packet_end_time;
        bool packetType = false;
        if (msg->data[1205] == 0x01) {
            int packet_interval_time = (current_packet_time - last_packet_time)/(POINTS_PER_PACKET/7);
            for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx += 7) {
                if (msg->data[point_idx] == 0xff && msg->data[point_idx + 1] == 0xaa &&
                    msg->data[point_idx + 2] == 0xbb) {
                    packetType = true;
                    if (last_packet_time > 1e-6) {
                        point_cloud_timestamp =
                                (packet_end_time  - packet_interval_time * ((POINTS_PER_PACKET - point_idx) / 7) ) /1000;
                    } else {
                        point_cloud_timestamp = current_packet_time /1000;
                    }
//                    point_cloud_timestamp =
//                            (packet_end_time - point_time_diff * 1e9 * ((POINTS_PER_PACKET - point_idx) / 7)) / 1000ull;
                }
                double point_time;
//                double point_time = packet_end_time * 1e-9 - point_time_diff * ((POINTS_PER_PACKET - point_idx) / 7 - 1);

                if (last_packet_time > 1e-6) {
                    point_time =
                            (packet_end_time  - packet_interval_time * ((POINTS_PER_PACKET - point_idx) / 7 - 1)) * 1e-9;
                } else {
                    point_time = current_packet_time * 1e-9;
                }

                if (msg->data[point_idx] < 255) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.vertical_line = msg->data[point_idx];
                    lidardata.azimuth = (msg->data[point_idx + 1] * 256 + msg->data[point_idx + 2]) * 0.01f;
                    lidardata.distance = (msg->data[point_idx + 3] * 65536 + msg->data[point_idx + 4] * 256 +
                                          msg->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                    lidardata.intensity = msg->data[point_idx + 6];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
                }
                if (packetType) {
                    //printf("---------------onesweep--------------------------\n");
                    if (publish_point_cloud) { publishPointCloud(); }
                    if (publish_laserscan) { publishLaserScan(); }
                    packetType = false;
                    sweep_data = lslidar_ch1w_msgs::LslidarChScanPtr(
                            new lslidar_ch1w_msgs::LslidarChScan());
                }
            }
        } else if (msg->data[1205] == 0x02) {
            int packet_interval_time = (current_packet_time - last_packet_time)/(POINTS_PER_PACKET_DOUBLE_ECHO/11);
            for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET_DOUBLE_ECHO; point_idx += 11) {
                if (msg->data[point_idx] == 0xff && msg->data[point_idx + 1] == 0xaa &&
                    msg->data[point_idx + 2] == 0xbb) {
                    packetType = true;
                    if (last_packet_time > 1e-6) {
                        point_cloud_timestamp =
                                (packet_end_time  - packet_interval_time * ((POINTS_PER_PACKET_DOUBLE_ECHO - point_idx) / 11) ) /1000;
                    } else {
                        point_cloud_timestamp = current_packet_time /1000;
                    }
//                    point_cloud_timestamp =
//                            (packet_end_time - point_time_diff * 1e9 * ((POINTS_PER_PACKET_DOUBLE_ECHO - point_idx) / 11)) / 1000ull;
//                ROS_INFO("pointcloud timestamp: %ld",point_cloud_timestamp);
                }
                double point_time;
//                double point_time = packet_end_time * 1e-9 - point_time_diff * ((POINTS_PER_PACKET - point_idx) / 7 - 1);

                if (last_packet_time > 1e-6) {
                    point_time =
                            (packet_end_time - packet_interval_time * ((POINTS_PER_PACKET_DOUBLE_ECHO - point_idx) / 11 - 1)) * 1e-9;
                } else {
                    point_time = current_packet_time * 1e-9;
                }
//                if(current_packet_time - last_packet_time>183040.000000 *1.5){
//                    ROS_INFO("current %ld, last %ld diff %ld",current_packet_time,last_packet_time,current_packet_time - last_packet_time);
//                }

                if (msg->data[point_idx] < 255) {
                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.vertical_line = msg->data[point_idx];
                    lidardata.azimuth = (msg->data[point_idx + 1] * 256 + msg->data[point_idx + 2]) * 0.01f;
                    lidardata.distance = (msg->data[point_idx + 3] * 65536 + msg->data[point_idx + 4] * 256 +
                                          msg->data[point_idx + 5]) * DISTANCE_RESOLUTION;
                    lidardata.intensity = msg->data[point_idx + 6];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);

                    memset(&lidardata, 0, sizeof(lidardata));
                    lidardata.vertical_line = msg->data[point_idx];
                    lidardata.azimuth = (msg->data[point_idx + 1] * 256 + msg->data[point_idx + 2]) * 0.01f;
                    lidardata.distance = (msg->data[point_idx + 7] * 65536 + msg->data[point_idx + 8] * 256 +
                                          msg->data[point_idx + 9]) * DISTANCE_RESOLUTION;
                    lidardata.intensity = msg->data[point_idx + 10];
                    lidardata.time = point_time;
                    convertCoordinate(lidardata);
                }
                if (packetType) {
                    //printf("---------------onesweep--------------------------\n");
                    if (publish_point_cloud) { publishPointCloud(); }
                    if (publish_laserscan) { publishLaserScan(); }
                    packetType = false;
                    sweep_data = lslidar_ch1w_msgs::LslidarChScanPtr(
                            new lslidar_ch1w_msgs::LslidarChScan());
                }
            }
        }
        last_packet_time = current_packet_time;
        return;
    }

} // namespace lslidar_ch_decoder

