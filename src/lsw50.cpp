/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSW50
@filename: lsw50.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           21-2-4      yao          new
*******************************************************/
#include "lsw50/lsw50.h"
#include <stdio.h>
#include <signal.h> 
#include<lsw50/difop.h>

namespace ls{
LSW50 * LSW50::instance()
{
  static LSW50 obj;
  return &obj;
}

LSW50::LSW50()
{
  int code = 0;
  initParam();
  pub_ = n_.advertise<sensor_msgs::LaserScan>(scan_topic_, 3);
  device_pub = n_.advertise<lsw50::difop>("difop_information", 100);
  
  serial_ = LSIOSR::instance(serial_port_, baud_rate_);
  code = serial_->init();
  if(code != 0)
  {
	  printf("open_port %s ERROR !\n", serial_port_.c_str());
	  ros::shutdown();
	  exit(0);
  }
  printf("open_port %s  OK !\n", serial_port_.c_str());

  recv_thread_ = new boost::thread(boost::bind(&LSW50::recvThread, this));
  pubscan_thread_ = new boost::thread(boost::bind(&LSW50::pubScanThread, this));
}

LSW50::~LSW50()
{

  is_shutdown_ = true;

  pubscan_thread_->interrupt();
  pubscan_thread_->join();
  pubscan_thread_ = NULL;
  delete pubscan_thread_;

  recv_thread_->interrupt();
  recv_thread_->join();

  recv_thread_ = NULL;
  delete recv_thread_;

  serial_->close();
  serial_ = NULL;
  delete serial_;
}

void LSW50::initParam()
{
  std::string scan_topic = "/scan";
  std::string frame_id = "laser_link";
  std::string port = "/dev/ttyUSB0";
  ros::NodeHandle nh("~");
  nh.param("scan_topic", scan_topic_, scan_topic);
  nh.param("frame_id", frame_id_, frame_id);
  nh.param("serial_port", serial_port_, port);
  nh.param("baud_rate", baud_rate_, 921600);
  nh.param("min_range", min_range, 0.3);
  nh.param("max_range", max_range, 100.0);
  nh.param("angle_disable_min", angle_disable_min,0.0);
  nh.param("angle_disable_max", angle_disable_max,0.0);
  while(angle_disable_min<0)	angle_disable_min+=360;
  while(angle_disable_max<0)	angle_disable_max+=360;
  while(angle_disable_min>360)	angle_disable_min-=360;
  while(angle_disable_max>360)	angle_disable_max-=360;
  if(angle_disable_max == angle_disable_min ){
	angle_able_min = 0;
	angle_able_max = 360;
  }
  else{
	if(angle_disable_min<angle_disable_max && angle_disable_min !=0.0){
		angle_able_min = angle_disable_max;
		angle_able_max = angle_disable_min+360;
	}
	if (angle_disable_min<angle_disable_max && angle_disable_min == 0.0){
		angle_able_min = angle_disable_max;
		angle_able_max = 360;
	}
	if (angle_disable_min>angle_disable_max ){
		angle_able_min = angle_disable_max; 
		angle_able_max = angle_disable_min; 
	}
  }
  is_shutdown_ = false;
  scan_points_.resize(2000);
}

int LSW50::getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration)
{
  boost::unique_lock<boost::mutex> lock(mutex_);
  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;
  scan_duration = (time_ - pre_time_).toSec();
}

int LSW50::getVersion(std::string &version)
{
  version = "w5.0";
  return 0;
}


void LSW50::recvThread()
{  
  int link_time = 0;
  double degree;
  double last_degree = 0.0;
  
  boost::posix_time::ptime t1,t2;
  t1 = boost::posix_time::microsec_clock::universal_time();
  
  while(!is_shutdown_&&ros::ok()){
		unsigned char * bytes = new unsigned char[100];
		int count = serial_->read(bytes, 100);
		if(count <= 0) 
			link_time++;
		else
		{
			link_time = 0;
			baud_rate_testing++;
		}
		if(link_time > 10000)
		{
			serial_->close();
			int ret = serial_->init();
			if(ret < 0)
			{
				ROS_WARN("serial open fail");
				usleep(200000);
			}
			link_time = 0;
		}
		if(baud_rate_testing>800)
		{
			baud_rate_testing = 0;
			ROS_WARN("The packet header cannot be detected. Please check the serial port and Linux kernel version.");
		}	
		for (int i = 0; i < count; i++)
		{	
			int flag = 0;
			int k = bytes[i];
			int y = bytes[i + 1];

			if (k == 0xfe && y == 0xff)
			{	
				int k_1 = 0;
				int y_1 = 0;

				if(i<count-3){
					k_1 = bytes[i + 2];
					y_1 = bytes[i + 3];
				}
				else	
					break;
				point_num = k_1*256+y_1;
				data_len_ = point_num*4+7;
  				points_size_ = data_len_ - count;
  				unsigned char * packet_bytes = new unsigned char[data_len_];
				memcpy(packet_bytes, bytes + i, count - i);

				for(int read_num = 1 ; read_num <= points_size_ / 100;read_num++) {
					int read_length = serial_->read(packet_bytes + 100*read_num - i, 100);
					if(read_length != 100)	
					{
						int read_length_2 = serial_->read(packet_bytes + 100*read_num - i + read_length, 100-read_length);
						if(read_length_2 != 100-read_length)	 	
							serial_->read(packet_bytes + 100*read_num - i + read_length+read_length_2, 100-read_length_2-read_length);
					}
				}

				serial_->read(packet_bytes + (data_len_- points_size_ % 100), points_size_ % 100);
				if (packet_bytes[data_len_ -2] != 0xcc || packet_bytes[data_len_ -1] != 0xdd)					break;
				flag = 1;
				baud_rate_testing = 0;
				for (size_t num = 0; num < point_num; num++)
				{
					int angle_high = packet_bytes[num * 4 + 4];
					int angle_low = packet_bytes[num * 4 + 5];
				
					int range_high = packet_bytes[num * 4 + 6];
					int range_low = packet_bytes[num * 4 + 7];

					scan_points_[num].degree = double(angle_high * 256 + (angle_low)) / 100.f;
					scan_points_[num].range = double(range_high * 256 + (range_low)) / 1000.f;
				}

				for(int k=0;k<scan_points_.size();k++)
				{	
					if(angle_able_max > 360)
					{	
						if((360-scan_points_[k].degree) > (angle_able_max - 360) && (360-scan_points_[k].degree) < angle_able_min)
							scan_points_[k].range = 0;
					}
					else 
					{
						if((360-scan_points_[k].degree) > angle_able_max || (360-scan_points_[k].degree) < angle_able_min)
							scan_points_[k].range = 0;
					}
					
					if(scan_points_[k].range < min_range || scan_points_[k].range > max_range)
						scan_points_[k].range = 0;
				}
				boost::unique_lock<boost::mutex> lock(mutex_);
				scan_points_bak_.resize(scan_points_.size());
				scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
				for(int k=0; k<scan_points_.size(); k++)
				{
					scan_points_[k].range = 0;
					scan_points_[k].degree = 0;
				}
				pre_time_ = time_;
				lock.unlock();
				pubscan_cond_.notify_one();
				time_ = ros::Time::now();
				
			}
			if(flag)
				break;
		}
    }
}

void LSW50::pubScanThread()
{
  bool wait_for_wake = true;
  boost::unique_lock<boost::mutex> lock(pubscan_mutex_);

  while (ros::ok() && !is_shutdown_)
  {
    while (wait_for_wake)
    {
      pubscan_cond_.wait(lock);
      wait_for_wake = false;
    }
    std::vector<ScanPoint> points;
    ros::Time start_time;
    float scan_time;
    this->getScan(points, start_time, scan_time);
    int count = point_num;
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = start_time;
	msg.angle_min = -0.75 * M_PI ;
    msg.angle_max = 0.75 * M_PI ;
	msg.angle_increment = 1.5 * M_PI / count;
    msg.range_min = min_range;
    msg.range_max = max_range;
    msg.ranges.resize(count);
    msg.intensities.resize(count);
    msg.scan_time = scan_time;
    msg.time_increment = scan_time / (double)(count - 1);
	for(int k=0; k<count; k++)
	{
		msg.ranges[k] = std::numeric_limits<float>::infinity();
        msg.intensities[k] = 0;
	}

	for (int i = 0; i < count; i++) {

		if (points[i].range == 0.0) 
			msg.ranges[count-i-1] = std::numeric_limits<float>::infinity();

		else {
			double dist = points[i].range;
			msg.ranges[count-i-1] = (float) dist;
		}
		msg.intensities[count-i-1] = 0;
    }
	
    pub_.publish(msg);
    wait_for_wake = true;
  }
}

}

void handleSig(int signo)
{
  printf("handleSig\n");
  ros::shutdown();
  exit(0);
}

int main(int argv, char **argc)
{
  signal(SIGINT, handleSig);
  signal(SIGTERM, handleSig);
  ros::init(argv, argc, "lsw50");
 
  ls::LSW50* lsw50 = ls::LSW50::instance();

  ros::spin();
  return 0;
}
