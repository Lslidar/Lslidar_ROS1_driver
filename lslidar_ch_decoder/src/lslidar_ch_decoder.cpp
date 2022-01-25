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
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
    publish_point_cloud(true),
    is_first_sweep(true),
    last_azimuth(0.0),
    sweep_start_time(0.0),
    packet_start_time(0.0),
    sweep_data(new lslidar_ch_msgs::LslidarChScan())
    {
    return;
}

bool LslidarChDecoder::loadParameters() {
    pnh.param<double>("min_range", min_range, 0.5);
    pnh.param<double>("max_range", max_range, 100.0);
    pnh.param<double>("frequency", frequency, 10.0);
    pnh.param<string>("lslidar_point_cloud", lslidar_point_cloud, "lslidar_point_cloud");
    pnh.param<bool>("publish_point_cloud", publish_point_cloud, true);
    pnh.param<bool>("apollo_interface", apollo_interface, false);
    pnh.param<int>("return_model", return_model_, 0);
    pnh.param<string>("frame_id", frame_id, "lslidar");
    pnh.param<bool>("use_gps_ts", use_gps_ts, false);
    ROS_INFO("Using GPS timestamp or not %d", use_gps_ts);

    //Vertical Angle initalization
    for(int i = 0; i < 4; i++){
      vertical_angle[i] = 0.0;
    }

    lidarpoint_count = 0;

    return true;
}

bool LslidarChDecoder::createRosIO() {
    packet_sub = nh.subscribe<lslidar_ch_msgs::LslidarChPacket>(
                "lslidar_packet", 100, &LslidarChDecoder::packetCallback, this);

    difop_sub = nh.subscribe<lslidar_ch_msgs::LslidarChPacket>(
                "lslidar_packet_difop", 100, &LslidarChDecoder::difopCallback, this);

    sweep_pub = nh.advertise<lslidar_ch_msgs::LslidarChSweep>(
                "lslidar_sweep", 10);
    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
                lslidar_point_cloud, 10);
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
    if (use_gps_ts){
      point_cloud->header.stamp = static_cast<uint64_t>(sweep_start_time.toSec() * 1e6);
    }
    else{
      point_cloud->header.stamp = static_cast<uint64_t>(timestamp * 1e6);
    }

    size_t j;
    VPoint point;
    if(sweep_data->points.size() > 0){
      for (j = 1; j < sweep_data->points.size()-1; ++j) {

        point.x = sweep_data->points[j].x;
        point.y = sweep_data->points[j].y;
        point.z = sweep_data->points[j].z;
        point.intensity = sweep_data->points[j].intensity;
        point.lines = sweep_data->points[j].line;
        point_cloud->points.push_back(point);
        ++point_cloud->width;

      }

      sensor_msgs::PointCloud2 pc_msg;
      pcl::toROSMsg(*point_cloud, pc_msg);
      point_cloud_pub.publish(pc_msg);
    }
    return;
}

void LslidarChDecoder::decodePacket(const RawPacket* packet) {

  // Compute the values for each firing
  for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx++) {

    firings[point_idx].vertical_line=packet->points[point_idx].vertical_line;
    TwoBytes point_amuzith;
    point_amuzith.bytes[0] = packet->points[point_idx].azimuth_2;
    point_amuzith.bytes[1] = packet->points[point_idx].azimuth_1;
    firings[point_idx].azimuth=static_cast<double>(point_amuzith.value)*0.01*DEG_TO_RAD;
    ROS_INFO("point_amuzith %f",firings[point_idx].azimuth);

    ThreeBytes point_distance;
    point_distance.bytes[0] = packet->points[point_idx].distance_3;
    point_distance.bytes[1] = packet->points[point_idx].distance_2;
    point_distance.bytes[2] = packet->points[point_idx].distance_1;
    firings[point_idx].distance=static_cast<double>(point_distance.value) * DISTANCE_RESOLUTION;
    firings[point_idx].intensity=packet->points[point_idx].intensity;

  }
  return;
}


int LslidarChDecoder::checkPacketValidity(const RawPacket* packet) {

  for (size_t blk_idx = 0; blk_idx < POINTS_PER_PACKET; blk_idx++) {
    if ((packet->points[blk_idx].vertical_line==0xff)&&(packet->points[blk_idx].azimuth_1 ==0xaa)&&
        (packet->points[blk_idx].azimuth_2 ==0xbb)){

      return true;
    }
  }
  return false;
}

int LslidarChDecoder::ConvertCoordinate(struct Firing lidardata)
{
  double x=0.0,y=0.0,z=0.0;

  // Check if the point is valid.
  if (!isPointInRange(lidardata.distance)){
    return -1;
  }

  unsigned int line_angle = lidardata.vertical_line;
 // unsigned int mask = ~(~0 << 4);
 // unsigned int vert_angle_select = line_angle & mask;
  double vert_angle;

/*  if(0x0 == line_angle){
    vert_angle = 0;
  }else if(0x1 == line_angle){
    vert_angle = 0.33;
  }else if(0x2 == line_angle){
    vert_angle = 0.66;
  }else if(0x3 == line_angle){
    vert_angle = 0.99;
  }*/
  vert_angle = line_angle* 0.33 * DEG_TO_RAD;

  // Convert the point to xyz coordinate
  x = lidardata.distance * cos(vert_angle) * cos(lidardata.azimuth);
  y = lidardata.distance * cos(vert_angle) * sin(lidardata.azimuth);
  z = lidardata.distance * sin(vert_angle);
  //printf("vert: %f, azimuth = %f\n", vert_angle, lidardata.azimuth);

  double x_coord = x;
  double y_coord = y;
  double z_coord = z;
  double time =0.0;

  sweep_data->points.push_back(lslidar_ch_msgs::LslidarChPoint());
  lslidar_ch_msgs::LslidarChPoint& new_point =		// new_point 为push_back最后一个的引用
      sweep_data->points[sweep_data->points.size()-1];

  // Pack the data into point msg
  new_point.time = time;
  new_point.x = x_coord;
  new_point.y = y_coord;
  new_point.z = z_coord;
  //new_point.vertical_angle = verticalLineToAngle(lidardata.vertical_line);
  new_point.vertical_angle = line_angle * 0.33;
  new_point.azimuth = lidardata.azimuth;
  new_point.distance = lidardata.distance;
  new_point.intensity = lidardata.intensity;
  new_point.line = lidardata.vertical_line;

  return 0;
}

void LslidarChDecoder::difopCallback(
    const lslidar_ch_msgs::LslidarChPacketConstPtr& msg) {

  const uint8_t* data = &msg->data[0];

  // check header
  if (data[0] != 0xa5 || data[1] != 0xff || data[2] != 0x00 || data[3] != 0x5a)
  {
    return;
  }

  double vert_angle1 = (data[662] * 256 + data[663]);
  if(vert_angle1 > 32767){
    vert_angle1 = vert_angle1 - 65535;
  }
  vertical_angle[0] = vert_angle1;

  double vert_angle2 = (data[664] * 256 + data[665]);
  if(vert_angle2 > 32767){
    vert_angle2 = vert_angle2 - 65535;
  }
  vertical_angle[1] = vert_angle2;

  double vert_angle3 = (data[666] * 256 + data[667]);
  if(vert_angle3 > 32767){
    vert_angle3 = vert_angle3 - 65535;
  }
  vertical_angle[2] = vert_angle3;

  double vert_angle4 = (data[668] * 256 + data[669]);
  if(vert_angle4 > 32767){
    vert_angle4 = vert_angle4 - 65535;
  }
  vertical_angle[3] = vert_angle4;
}


void LslidarChDecoder::packetCallback(
    const lslidar_ch_msgs::LslidarChPacketConstPtr& msg) {

  struct Firing lidardata;
  struct Firing lidardata_d;

  // Check if the packet is valid and find the header of frame
  bool packetType=false;
  sweep_start_time = msg->header.stamp;

  for (size_t point_idx = 0; point_idx < POINTS_PER_PACKET; point_idx += 7) {

    if((msg->data[point_idx] == 0xff) && (msg->data[point_idx+1] == 0xaa) && (msg->data[point_idx+2] == 0xbb)){
      lidarpoint_count++;
      packetType = true;
    }

    if(msg->data[point_idx] < 255){
        memset(&lidardata,0,sizeof(lidardata));
        lidardata.vertical_line = (msg->data[point_idx] & 0xf0) >> 4;
        lidardata.azimuth = (msg->data[point_idx+1]*256 + msg->data[point_idx+2])/100.f * DEG_TO_RAD;
        lidardata.distance = (msg->data[point_idx+3]*65536 + msg->data[point_idx+4]*256 + msg->data[point_idx+5]) * DISTANCE_RESOLUTION;
        lidardata.intensity = msg->data[point_idx+6];
        // int face_num =  (msg->data[point_idx] & 0xf0)>> 4;
        //if(face_num != 0) continue;
        ConvertCoordinate(lidardata);
    }

    if ((true == packetType) && (4 == lidarpoint_count))
    {
      publishPointCloud();
      lidarpoint_count = 0;
      packetType = false;
      sweep_data = lslidar_ch_msgs::LslidarChScanPtr(
            new lslidar_ch_msgs::LslidarChScan());
    }
  }

  return;
}

} // end namespace lslidar_ch_decoder

