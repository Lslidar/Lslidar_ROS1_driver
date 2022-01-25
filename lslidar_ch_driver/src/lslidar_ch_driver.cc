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

#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <lslidar_ch_driver/lslidar_ch_driver.h>

namespace lslidar_ch_driver {

LslidarChDriver::LslidarChDriver(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
    socket_id(-1){
    gps_count = 0;
    GPSCurrentTS = 0;
    return;
}

LslidarChDriver::~LslidarChDriver() {

  if(NULL == difop_thread_){
    difop_thread_->interrupt();
    difop_thread_->join();
  }
  return;
}

bool LslidarChDriver::loadParameters() {

  pnh.param("frame_id", frame_id, std::string("lslidar"));
  pnh.param("lidar_ip", lidar_ip_string, std::string("192.168.1.222"));
  pnh.param<int>("device_port", UDP_PORT_NUMBER,2368);
  pnh.param<bool>("add_multicast", add_multicast, false);
  pnh.param("group_ip", group_ip_string, std::string("234.2.3.2"));
  pnh.param("msop_port", msop_udp_port, (int)MSOP_DATA_PORT_NUMBER);
  pnh.param("difop_port", difop_udp_port, (int)DIFOP_DATA_PORT_NUMBER);
  pnh.param("time_synchronization", time_synchronization_, false);

  inet_aton(lidar_ip_string.c_str(), &lidar_ip);
  ROS_INFO_STREAM("Opening UDP socket: address " << lidar_ip_string);
  if(add_multicast) ROS_INFO_STREAM("Opening UDP socket: group_address " << group_ip_string);
  ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);

  return true;
}

bool LslidarChDriver::createRosIO() {

  // ROS diagnostics
  diagnostics.setHardwareID("Lslidar_Ch");
  // ch publishs 20*16 thousands points per second.
  // Each packet contains 12 blocks. And each block
  // contains 32 points. Together provides the
  // packet rate.
  const double diag_freq = 32*20000.0 / 200;
  diag_max_freq = diag_freq;
  diag_min_freq = diag_freq;
  //ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic.reset(new TopicDiagnostic(
                     "lslidar_packets", diagnostics,
                     FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
                     TimeStampStatusParam()));

  // Output
  packet_pub = nh.advertise<lslidar_ch_msgs::LslidarChPacket>(
        "lslidar_packet", 100);

  difop_pub = nh.advertise<lslidar_ch_msgs::LslidarChPacket>(
        "lslidar_packet_difop", 100);

  msop_input_.reset(new lslidar_ch_driver::InputSocket(pnh,msop_udp_port));
  difop_input_.reset(new lslidar_ch_driver::InputSocket(pnh,difop_udp_port));
  difop_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&LslidarChDriver::difopPoll, this)));

  if (time_synchronization_)
  {
    output_sync_ = nh.advertise<sensor_msgs::TimeReference>("sync_header", 1);
  }

  return true;
}

bool LslidarChDriver::initialize() {

    this->initTimeStamp();

    if (!loadParameters()) {
        ROS_ERROR("Cannot load all required ROS parameters...");
        return false;
    }

    if (!createRosIO()) {
        ROS_ERROR("Cannot create all ROS IO...");
        return false;
    }

    return true;
}

bool LslidarChDriver::polling()
{
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    lslidar_ch_msgs::LslidarChPacketPtr packet(
                new lslidar_ch_msgs::LslidarChPacket());

    // Since the lslidar delivers data at a very high rate, keep
    // reading and publishing scans as fast as possible.
    while (true)
    {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(packet);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
    }

    if(gps_count == 1){
         GPSCurrentTS = GPSCountingTS;
         gps_count = 0;
    }

    // notify diagnostics that a message has been published, updating
    // its status
    diag_topic->tick(packet->stamp);
    diagnostics.update();

    if (time_synchronization_ && (GPSCurrentTS > 0))
    {
      sensor_msgs::TimeReference sync_header;

      // it is already the msop msg
      // use the first packets
      lslidar_ch_msgs::LslidarChPacket pkt = *packet;
      uint64_t packet_timestamp;
      packet_timestamp = (pkt.data[1200]  +
          pkt.data[1201] * pow(2, 8) +
          pkt.data[1202] * pow(2, 16) +
          pkt.data[1203] * pow(2, 24)) * 1e3; //ns

      timeStamp = ros::Time(GPSCurrentTS, packet_timestamp);// s,ns
      sync_header.header.stamp = timeStamp;
      output_sync_.publish(sync_header);
      packet->header.stamp = timeStamp;

    }

    packet_pub.publish(*packet);
    return true;
}

void LslidarChDriver::initTimeStamp(void)
{
    int i;

    for(i = 0;i < 10;i ++)
    {
        this->packetTimeStamp[i] = 0;
    }
    this->pointcloudTimeStamp = 0;

    this->timeStamp = ros::Time(0.0);
}

void LslidarChDriver::difopPoll(void)
{
  lslidar_ch_msgs::LslidarChPacketPtr difop_packet(
              new lslidar_ch_msgs::LslidarChPacket());

  // reading and publishing scans as fast as possible.
  while (ros::ok())
  {
    // keep reading
    int rc = difop_input_->getPacket(difop_packet);
    if (rc == 0)
    {
      getFPGA_GPSTimeStamp(difop_packet);
      lslidar_ch_msgs::LslidarChPacket pkt = *difop_packet;
      difop_pub.publish(*difop_packet);

    }else if(rc < 0){
      return ;
    }
    ros::spinOnce();
  }
}

void LslidarChDriver::getFPGA_GPSTimeStamp(lslidar_ch_msgs::LslidarChPacketPtr &packet)
{
    unsigned char head2[] = {packet->data[0],packet->data[1],packet->data[2],packet->data[3]};

    if(head2[0] == 0xA5 && head2[1] == 0xFF)
    {
        if(head2[2] == 0x00 && head2[3] == 0x5A)
        {
            this->packetTimeStamp[4] = packet->data[41];
            this->packetTimeStamp[5] = packet->data[40];
            this->packetTimeStamp[6] = packet->data[39];
            this->packetTimeStamp[7] = packet->data[38];
            this->packetTimeStamp[8] = packet->data[37];
            this->packetTimeStamp[9] = packet->data[36];

            cur_time.tm_sec = this->packetTimeStamp[4];
            cur_time.tm_min = this->packetTimeStamp[5];
            cur_time.tm_hour = this->packetTimeStamp[6];
            cur_time.tm_mday = this->packetTimeStamp[7];
            cur_time.tm_mon = this->packetTimeStamp[8]-1;
            cur_time.tm_year = this->packetTimeStamp[9]+2000-1900;
            this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time));
            if (GPSCountingTS != this->pointcloudTimeStamp)
            {
                GPSCountingTS = this->pointcloudTimeStamp;
                gps_count = 1;
            }

        }
    }
}

} // namespace lslidar_driver
