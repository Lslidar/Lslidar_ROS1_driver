/*
 * This file is part of lslidar_n301 driver.
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

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <lslidar_n301_driver/lslidar_n301_driver.h>

extern volatile sig_atomic_t flag;

namespace lslidar_n301_driver {

LslidarN301Driver::LslidarN301Driver(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
	first_time(true),
    last_degree(0.0),
	timeout_flags(false),
	socket_id_difop(-1),
    socket_id(-1){
    return;
}

LslidarN301Driver::~LslidarN301Driver() {
    (void) close(socket_id);
	(void) close(socket_id_difop);
    return;
}

bool LslidarN301Driver::loadParameters() {

	pnh.param("device_ip", device_ip_string, std::string("192.168.1.222"));
	pnh.param<int>("msop_port", UDP_PORT_NUMBER, 2368);
	pnh.param<int>("agreement_type", agreement_type, 1);
	pnh.param<int>("difop_port", UDP_DIFOP_PORT_NUMBER, 2369);
	pnh.param<bool>("add_multicast", add_multicast, false);
	pnh.param<std::string>("group_ip", group_ip, "224.1.1.2");

    pnh.param<double>("min_range", min_range, 0.3);
    pnh.param<double>("max_range", max_range, 100.0);
    pnh.param<int>("start_angle", start_angle,0);
    pnh.param<int>("end_angle", end_angle, 360);
    pnh.param<bool>("use_gps_ts", use_gps_ts, false);
    pnh.param<std::string>("child_frame_id", child_frame_id, "laser_link");
    pnh.param<std::string>("scan_topic", scan_topic_, "scan");
    
    while(end_angle<0)          end_angle +=360;
    while(end_angle>360)        end_angle -=360;
    while(start_angle<0)        start_angle +=360;
    while(start_angle>360)      start_angle -=360;
    if(start_angle>end_angle)   end_angle +=360;
    if(start_angle==end_angle)   
    {
        start_angle =0;
        end_angle =360;
    }
    inet_aton(device_ip_string.c_str(), &device_ip);
	ROS_INFO_STREAM("Opening UDP socket: address " << device_ip_string);
	
	if(agreement_type == 1) 
    {   
        block_point_num = 15;
        DISTANCE_RESOLUTION = 0.004;
    }
    else 
    {
        block_point_num = 1;
        DISTANCE_RESOLUTION = 0.002;
    }
    idx = 0;
    count_num = 0;
    scan_points_.resize(4000);

	return true;
}

bool LslidarN301Driver::createRosIO() {

  // ROS diagnostics
  diagnostics.setHardwareID("Lslidar_N301");

  // n301 publishs 20*16 thousands points per second.
  // Each packet contains 12 blocks. And each block
  // contains 30 points. Together provides the
  // packet rate.
  const double diag_freq = 16*20000.0 / (12*30);
  diag_max_freq = diag_freq;
  diag_min_freq = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
	  diag_topic.reset(new TopicDiagnostic(
						 "lslidar_packets", diagnostics,
						 FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
						 TimeStampStatusParam()));
	int hz = 1;
	if(agreement_type != 1) hz = 14;	
	double packet_rate = 60*hz+1;
    std::string dump_file;
    pnh.param("pcap",dump_file,std::string(""));
    if(dump_file !="")
    {
        msop_input_.reset(new lslidar_n301_driver::InputPCAP(pnh,UDP_PORT_NUMBER,packet_rate,dump_file));
    }else{
        msop_input_.reset(new lslidar_n301_driver::InputSocket(pnh,UDP_PORT_NUMBER));
    }

    scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic_, 200);

	return true;
}

bool LslidarN301Driver::initialize() {
    if (!loadParameters()) {
        ROS_ERROR("Cannot load all required ROS parameters...");
        return false;
    }

    if (!createRosIO()) {
        ROS_ERROR("Cannot create all ROS IO...");
        return false;
    }

    ROS_INFO("Initialised lslidar n301 without error");
    return true;
}

int LslidarN301Driver::getDifopPacket(lslidar_n301_msgs::LslidarN301PacketPtr& packet) {
    double time1 = ros::Time::now().toSec();

    struct pollfd fds[1];
    fds[0].fd = socket_id_difop;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 3000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (true)
    {
        do {
            int retval = poll(fds, 1, POLL_TIMEOUT);
            if (retval < 0)             // poll() error?
            {
                if (errno != EINTR)
                    ROS_ERROR("poll() error: %s", strerror(errno));
                return 1;
            }
            if (retval == 0)            // poll() timeout?
            {
                //ROS_WARN("lslidar_difop poll() timeout");
                return 1;
            }
            if ((fds[0].revents & POLLERR)
                    || (fds[0].revents & POLLHUP)
                    || (fds[0].revents & POLLNVAL)) // device error?
            {
                ROS_ERROR("poll() reports lslidar error");
                return 1;
            }
        } while ((fds[0].revents & POLLIN) == 0);

        // Receive packets that should now be available from the
        // socket using a blocking read.
        ssize_t nbytes = recvfrom(socket_id_difop, &packet->data[0], PACKET_SIZE,  0,
                (sockaddr*) &sender_address, &sender_address_len);
        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                perror("recvfail");
                ROS_INFO("recvfail");
                return 1;
            }
        }
        else if ((size_t) nbytes == PACKET_SIZE)
        {
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if( device_ip_string != "" && sender_address.sin_addr.s_addr != device_ip.s_addr )
			{
                continue;
			}
            else
                break; //done
        }
    }
    return 0;
}

bool LslidarN301Driver::polling()
{
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    lslidar_n301_msgs::LslidarN301PacketPtr packet(
                new lslidar_n301_msgs::LslidarN301Packet());
				
	std_msgs::Byte msg;
	struct timeval tv;
	int last_usec,now_usec;
	
	while (true)
    {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(packet);
        if (rc == 0) break;       // got a full packet?
		if(rc == 1)
		{
			msg.data = 2;//no link
		}
        if (rc < 0) return false; // end of file reached?
    }
	
	//Determine the protocol type based on the packet interval
	//////////////////////////////////////////
	if(first_time)
	{
		first_time = false;
		gettimeofday(&tv,NULL);
		last_usec = tv.tv_sec * 1000 + tv.tv_usec/1000;
		
		for(int i=0;i<200;i++)
		{
			while (true)
			{
				// keep reading until full packet received
				int rc = msop_input_->getPacket(packet);
				if (rc == 0) break;       // got a full packet?
				if (rc < 0) return false; // end of file reached?
			}
			if(i == 1)
			{
				time_t curTime = time(NULL);
				struct tm *curTm = localtime(&curTime);
				ROS_INFO("lslidar start in %d:%d:%d:%d:%d:%d", curTm->tm_year+1900, 
				curTm->tm_mon+1, curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);	
			}
		}
		
		gettimeofday(&tv,NULL);
		now_usec = tv.tv_sec * 1000 + tv.tv_usec/1000;
		
		if(now_usec - last_usec > 2000)//100*18ms/2
		{
			msg.data = 1;//v3.0/v4.0
		}
	}
	//////////////////////////////////////////////
	

    // publish message using time of last packet read
    ROS_DEBUG("Publishing a full lslidar scan.");
	
	 if (packet->data[0] == 0xff || packet->data[1] == 0xee)
	 {
        const RawPacket* raw_packet = (const RawPacket*) (&(packet->data[0]));
        decodePacket(raw_packet);
	 }
    diag_topic->tick(packet->stamp);
    diagnostics.update();

    return true;
}


void LslidarN301Driver::decodePacket(const RawPacket* packet) {

    for(int i = 0; i < 12; i++) {
        if(packet->blocks[i].header != UPPER_BANK)  continue;

        double azimuth = (double)(packet->blocks[i].rotation/100.0);
        double azimuth_2 = 0;

        if(i<11)    azimuth_2 = (double)((packet->blocks[i+1].rotation)/100.0);
        else        azimuth_2 = (double)((packet->blocks[i-1].rotation)/100.0);
        if(azimuth > 360)   azimuth-=360;
        if(azimuth_2 > 360)   azimuth_2-=360;

        double azimuth_diff = fabs(azimuth - azimuth_2);
        if(azimuth_diff>180)   azimuth_diff = fabs(360-azimuth_diff);
        azimuth_diff = azimuth_diff / block_point_num/2.0;

        for(int j = 0; j < 2; j++){
            for(int k = 0; k < block_point_num; k++){
                double point_azimuth = (azimuth+(j*block_point_num+k)*azimuth_diff);
                if(point_azimuth >= 360)   point_azimuth-=360;
                
                double point_distance = (double)(( packet->blocks[i].data[j*48+k*3+1]*256+packet->blocks[i].data[j*48+k*3])*DISTANCE_RESOLUTION);
                int point_Intensity = packet->blocks[i].data[j*48+k*3+2];
                scan_points_[idx].azimuth = point_azimuth;
                scan_points_[idx].distance = point_distance;
                scan_points_[idx].intensity = point_Intensity;
                idx++;
                
                if (point_azimuth < last_degree && last_degree > 350 && point_azimuth < 10) 	
                {   
                    //printf("last_degree = %f\n",last_degree);
                    count_num = idx;
                    idx = 0;
                    for(int z=0;z<scan_points_.size();z++)
                    {	
                        if(scan_points_[z].distance < min_range || scan_points_[z].distance > max_range)
                            scan_points_[z].distance = 0;
                    }

                    publishScan();

                    for(int z=0; z<scan_points_.size(); z++)
                    {
                        scan_points_[z].azimuth = 0;
                        scan_points_[z].distance = 0;
                        scan_points_[z].intensity = 0;
                    }
                }
                last_degree = point_azimuth;
            }       
        }

        if(use_gps_ts)
        {
            pTime.tm_year = packet->blocks[i].data[45]+2000-1900;
            pTime.tm_mon  = packet->blocks[i].data[46]-1;
            pTime.tm_mday = packet->blocks[i].data[47];
            pTime.tm_hour = packet->blocks[i].data[93];
            pTime.tm_min  = packet->blocks[i].data[94];
            pTime.tm_sec  = packet->blocks[i].data[95];
            packet_timestamp = (packet->time_stamp_yt[0] +
                    packet->time_stamp_yt[1] * pow(2, 8) +
                    packet->time_stamp_yt[2] * pow(2, 16) +
                    packet->time_stamp_yt[3] * pow(2, 24)) * 1e3;
            sweep_end_time_gps = get_gps_stamp(pTime);
		    sweep_end_time_hardware = packet_timestamp%1000000000;
        }
        /*
        if (azimuth < last_degree && last_degree >350 && azimuth<10) 	
        {   //printf("azimuth = %f     ",azimuth);
            //printf("last_degree = %f\n",last_degree);
            count_num = idx;
            idx = 0;
            for(int k=0;k<scan_points_.size();k++)
            {	
                if(scan_points_[k].distance < min_range || scan_points_[k].distance > max_range)
                    scan_points_[k].distance = 0;
            }

            publishScan();

            for(int k=0; k<scan_points_.size(); k++)
            {
                scan_points_[k].azimuth = 0;
                scan_points_[k].distance = 0;
                scan_points_[k].intensity = 0;
            }
        }
        last_degree = azimuth;
        */
    }
    // resolve the timestamp in the end of packet
    // ROS_DEBUG("nsec part: %lu", packet_timestamp);
	
    return;
}

uint64_t LslidarN301Driver::get_gps_stamp(struct tm t){

   uint64_t ptime =static_cast<uint64_t>(timegm(&t));
   return ptime;
}

void LslidarN301Driver::publishScan()
{
    sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
	
    scan->header.frame_id = child_frame_id;
	if (use_gps_ts){
		scan->header.stamp = ros::Time(sweep_end_time_gps, sweep_end_time_hardware);
	}
    else{
		scan->header.stamp = ros::Time::now();
	}
	if(count_num <= 1)	return;
    int scan_num = ceil(count_num*(end_angle - start_angle)/360 + 1);	
    //printf("scan_num == %d\n",scan_num);
    scan->time_increment = 0.1/(double)(count_num - 1);
    scan->scan_time= 0.1;
	if(end_angle >360){
        scan->angle_min = 2.0*M_PI*(start_angle-360)/360;
        scan->angle_max = 2.0*M_PI*(end_angle-360)/360;    
    }
    else{
        scan->angle_min = 2.0*M_PI*start_angle/360;
        scan->angle_max = 2.0*M_PI*end_angle/360;
    }
    scan->angle_increment = 2 * M_PI / count_num;
    scan->range_min = min_range;
    scan->range_max = max_range;
    scan->ranges.reserve(scan_num);
    scan->ranges.assign(scan_num, std::numeric_limits<float>::infinity());
    scan->intensities.reserve(scan_num);
    scan->intensities.assign(scan_num, std::numeric_limits<float>::infinity());

	int start_num = floor(start_angle * count_num / 360);
	int end_num = floor(end_angle * count_num / 360);
	for(uint16_t i = 0; i < count_num; i++)
	{
		int point_idx = round((360 - scan_points_[i].azimuth) * count_num / 360);
		if(point_idx<(end_num-count_num))
			point_idx += count_num;
		point_idx =  point_idx - start_num;
		
		if(point_idx < 0 || point_idx >= scan_num) 
			continue;
		if (scan_points_[i].distance == 0.0) {
			scan->ranges[point_idx] = std::numeric_limits<float>::infinity();
            scan->intensities[point_idx] = 0;
		}
		else{
			scan->ranges[point_idx] = scan_points_[i].distance;
            scan->intensities[point_idx] = scan_points_[i].intensity;
		}
		
		
	}
	count_num = 0;

    scan_pub.publish(scan);
}

} // namespace lslidar_driver
