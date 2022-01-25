/*******************************************************
@company: Copyright (C) 2018, Leishen Intelligent System
@product: LS01B
@filename: ls01b.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           18-8-21     fu          new
@v1.5           19-4-16     tongsky     Add service to change resolution online
@v1.5           19-4-18     tongsky     Fix bug when switching resolution
@v2.0           19-05-15    tongsky     Add service to start and stop laser; Modify truncated laser data behavior in a smart way
@v2.1           19-07-29    tongsky     Add angle compensation to fix distortion around near obstacle
*******************************************************/
#include "ls01b_v2/ls01b.h"
#include <stdio.h>
#include <signal.h>

#define	MAX_BUF 350

namespace ls{
LS01B * LS01B::instance()
{
  static LS01B obj;
  return &obj;
}

LS01B::LS01B()
{
  int code = 0;
  initParam();
  pub_ = n_.advertise<sensor_msgs::LaserScan>(scan_topic_, 3);
  serial_ = LSIOSR::instance(serial_port_, baud_rate_);
  code = serial_->init();
  if(code != 0)
  {
	ros::shutdown();
	exit(0);
  }
  changeRes_client = n_.advertiseService("resolution_control", &LS01B::resServerCallback, this);
  start_client = n_.advertiseService("start_control", &LS01B::startCallback, this);
  recv_thread_ = new boost::thread(boost::bind(&LS01B::recvThread, this));
  pubscan_thread_ = new boost::thread(boost::bind(&LS01B::pubScanThread, this));
}

LS01B::~LS01B()
{
  printf("start LS01B::~LS01B()\n");
  stopRecvData();
  usleep(100000);
  stopRecvData();
  usleep(100000);
  stopScan();
  usleep(100000);
  stopScan();

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
  printf("end LS01B::~LS01B()\n");
}

void LS01B::initParam()
{
  std::string scan_topic = "/scan";
  std::string frame_id = "laser_link";
  std::string port = "/dev/ttyUSB0";
  ros::NodeHandle nh("~");
  nh.param("scan_topic", scan_topic_, scan_topic);
  nh.param("frame_id", frame_id_, frame_id);
  nh.param("serial_port", serial_port_, port);
  nh.param("baud_rate", baud_rate_, 460800);
  nh.param("angle_resolution", resolution_, 1.0);
  nh.param("robot_radius", robot_radius_, 0.2);
  nh.param("center_x", center_x_, 0.0);
  nh.param("center_y", center_y_, 0.0);
  nh.param("rpm", rpm_, 600);
  nh.param("special_range", special_range_, 0.0);
  nh.param("angle_compensate", flag_angle_compensate, true);

  scan_health_ = 0;
  is_shutdown_ = false;
  is_start_ = false;
  use_angle_ = true;
  start_switch = true;

  data_len_ = 180;
  points_size_ = 360 / resolution_;
  scan_points_.resize(points_size_);

  // different method to discard laser data
  int disable_tolerance;
  nh.param<int>("disable_tolerance", disable_tolerance, 5);
  nh.param<int>("truncated_mode", truncated_mode_, 0);
  std::vector<int> disable_angle_min_range, disable_angle_max_range, disable_angle_range_default;
  nh.param<std::vector<int>>("disable_min", disable_angle_min_range, disable_angle_range_default);
  nh.param<std::vector<int>>("disable_max", disable_angle_max_range, disable_angle_range_default);
  for(unsigned i=0; i < disable_angle_min_range.size(); i++)
  {
      ROS_INFO("truncated angle min is %d", disable_angle_min_range[i]);
      ROS_INFO("truncated angle max is %d", disable_angle_max_range[i]);
      disable_angle_min_range_.push_back(disable_angle_min_range[i]);
      disable_angle_max_range_.push_back(disable_angle_max_range[i]);
  }

  if (truncated_mode_ == 1)
  {
      ROS_INFO("truncated mode is specific angle ranges");
  }
  else if (truncated_mode_ == 2)
      ROS_INFO("truncated mode is radius limits");
  else if (truncated_mode_ == 3)
      ROS_INFO("truncated mode is radius and specific angle ranges");
  else
      ROS_INFO("truncated mode is disable");

  dynamic_reconfigure::Server<ls01b_v2::FilterConfig> *dsrv_;
  dsrv_ = new dynamic_reconfigure::Server<ls01b_v2::FilterConfig>(nh);
  dynamic_reconfigure::Server<ls01b_v2::FilterConfig>::CallbackType cb = boost::bind(&LS01B::DynParamCallback, this, _1, _2);
  dsrv_->setCallback(cb);
}

void
LS01B::DynParamCallback(ls01b_v2::FilterConfig &config, uint32_t level)
{
  ROS_INFO("reaction parameters reconfigure request received!");
  robot_radius_ = config.robot_radius;
  truncated_mode_ = config.truncated_mode;
  center_x_ = config.center_x;
  center_y_ = config.center_y;
  double sr = config.special_range;
  flag_angle_compensate = config.angle_compensate;
  
   ROS_INFO("Enable angle sr is %lf",sr);
  if (sr > 90)
  {
    special_range_ = std::numeric_limits<float>::infinity();
  } else
  {
      special_range_ = sr;
  }
  ROS_INFO("Enable angle compensate or not %d", flag_angle_compensate);
  ROS_INFO("current robot radius is %1.3f, centerx %1.3f, centery %1.3f", config.robot_radius, config.center_x, config.center_y);
  ROS_INFO("Truncated Mode is %d, special range for truncated laser data is %3.3f", truncated_mode_, special_range_);

}
bool LS01B::isHealth()
{
  return (scan_health_ < 0) ? false : true;
}

bool LS01B::resetHealth()
{
  scan_health_ = 0;
}

int LS01B::getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration)
{
   boost::unique_lock<boost::mutex> lock(mutex_);
  points.assign(scan_points_bak_.begin(), scan_points_bak_.end());
  scan_time = pre_time_;

  scan_duration = (time_ - pre_time_).toSec();
}

int LS01B::getVersion(std::string &version)
{
  version = "ls01b_v2_0";
  return 0;
}

double LS01B::getRPM()
{
  return real_rpm_;
}

int LS01B::startScan()
{
  printf("startScan\n");
  char data[3];
  data[0] = 0xa5;
  data[1] = 0x2c;
  data[2] = 0x0;
  int rtn = serial_->send((const char*)data, 2);
  if (rtn < 0)
  {
    printf("start scan error !\n");
    scan_health_ = -1;
    return rtn;
  }
  start_switch = true;
  return rtn;
}

int LS01B::stopScan()
{
  printf("stopScan\n");
  char data[2];
  data[0] = 0xa5;
  data[1] = 0x25;
  int rtn = serial_->send((const char *)data, 2);
  if (rtn < 0)
  {
    printf("stop scan error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  start_switch = false;
  return rtn;
}

int LS01B::setScanMode(bool is_continuous)
{
  printf("setScanMode is_continuous = %d\n", is_continuous);
  char data[2];
  data[0] = 0xa5;
  data[1] = 0x0;
  
  if (is_continuous){
    data[1] = 0x20;
  }
  else{
    data[1] = 0x22;
  }

  int rtn = serial_->send((const char *)data, 2);
  if (rtn < 0)
  {
    printf("error !\n");
    scan_health_ = -1;
    return rtn;
  }
  serial_->flushinput();
  is_start_ = true;
  usleep(200000);
  return rtn;
}

int LS01B::stopRecvData()
{
  printf("stopRecvData\n");
  char data[2];
  data[0] = 0xa5;
  data[1] = 0x21;

  int rtn = serial_->send((const char *)data, 2);
  if (rtn < 0)
  {
    printf("stopRecvData error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  return rtn;
}



int LS01B::switchData(bool use_angle)
{
  printf("switchData use_angle = %d\n", use_angle);
  char data[4];
  data[0] =  0xa5;
  data[1] = 0x0;
  data[2] = 0x0;
  data[3] = 0x0;

  if (use_angle){
    data[1] = 0x5c;
  }
  else{
    data[1] = 0x50;
  }

  int rtn = serial_->send((const char *)data, 4);
  if (rtn < 0)
  {
    printf("switchData error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;
  use_angle_ = use_angle;

  return rtn;
}

int LS01B::setMotorSpeed(int rpm)
{
  printf("setMotorSpeed rpm = %d\n", rpm);

  char data[4];
  data[0] = 0xa5;
  data[1] = 0x26;
  data[3] = (rpm & 0xff);
  data[2] = (rpm >> 8) & 0xff;

  int rtn = serial_->send((const char *)data, 4);
  if (rtn < 0)
  {
    printf("setMotorSpeed error !\n");
    scan_health_ = -1;
    return rtn;
  }
  is_start_ = false;

  return rtn;
}

int LS01B::setResolution(double resolution)
{
  printf("setResolution resolution = %f\n", resolution);

  is_start_ = false;
  char data[4];
  data[0] = 0xa5;
  data[1] = 0x30;
  data[2] = 0x00;

  if (resolution == 0.25){
    data[3] = 0x19;
    data_len_ = 180;
  }
  else if (resolution == 0.5){
    data[3] = 0x32;
    data_len_ = 90;
  }
  else if (resolution == 1.0){
    data[3] = 0x64;
    data_len_ = 45;
  }
  else
  {
    // resolution error
    scan_health_ = -2;
    return scan_health_;
  }

  resolution_ = resolution;
  points_size_ = 360/resolution;
  scan_points_.resize(points_size_);
  

  int rtn = serial_->send((const char *)data, 4);
  if (rtn < 0)
  {
    printf("setResolution error !\n");
    scan_health_ = -1;
    return rtn;
  }
  serial_->flushinput();
  return rtn;
}

bool LS01B::startCallback(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &res)
{
  bool cmd = req.data;
  if (cmd == start_switch)
  {
    res.success = false;
    if (cmd)
    {
      ROS_INFO("laser already start");
      res.message = "laser already start";
    }
    else
    {
      ROS_INFO("laser already stop");
      res.message = "laser already stop";
    }
  }
  else{
    res.success = true;
    if (cmd){
      startScan();
      usleep(100000);
      startScan();
      usleep(100000);
      setScanMode(true);
      res.message = "bring up laser";
    }
    else
    {
      stopRecvData();
      usleep(100000);
      stopScan();
      res.message = "shutdown laser";
    }
  }
  return true;
}

bool LS01B::resServerCallback(ls01b_v2::resolution::Request &req,
                   ls01b_v2::resolution::Response &res)
{
  double resolution = req.resolution.data;
  if (resolution_ == resolution)
  {
      ROS_INFO("The same resolution, no need to switch");
      res.status = true;
      return true;
  }
  if ((resolution == 0.25) | (resolution == 0.5)  | (resolution == 1.0))
  {
    ROS_WARN("User want to switch to resolution %1.2f degree", resolution);
  }
  else{
    ROS_WARN("Invalid angle resolution to set! Keep it same");
    res.status = false;
    return true;
  }
  stopRecvData();   // stop sending data
  usleep(100);
  setResolution(resolution);
  usleep(100000);
  setResolution(resolution);
  usleep(100000);
  setScanMode(true);
  res.status = true;
  serial_->flushinput();
  return true;
}

void LS01B::recvThread()
{
  uint8_t start_count = 0;
  unsigned int time = 0;
  unsigned int total;
  unsigned int Intercept;
  unsigned int i;
  int count;
  int last_angle;
  bool first_data = true;
  bool link_fail = false;
  char header[6];
  uint8_t temp_char;
  char * packet_bytes = new char[data_len_];
  char * all_bytes = new char[MAX_BUF];
  if (packet_bytes == NULL)
  {
    packet_bytes = NULL;
  }
  if (all_bytes == NULL)
  {
    all_bytes = NULL;
  }
  
  int cnt_switch_res_failed = 0;
  boost::posix_time::ptime t1,t2;
  t1 = boost::posix_time::microsec_clock::universal_time();
  while(!is_shutdown_&&ros::ok()){

    while(!is_start_){
      usleep(100000);
    }
	
    bool is_health = this->isHealth();
	
    if (!is_health)
    {
		serial_->close();
      this->resetHealth();
      serial_ = LSIOSR::instance(serial_port_, baud_rate_);
	  int code = serial_->init();
	  if(code != 0)
	  {
		usleep(200000);
		ROS_WARN("Serial open fail");
		scan_health_ = -3;
		continue;
	  }
      usleep(100000);
      run();
      continue;
    }
	
	
	if(first_data)
	{ 
		total = 0;
		Intercept = 0;
		i = 0;
		while(true)
		{
			count = serial_->read(&all_bytes[0], 1);

			if ((int)(all_bytes[0]&0xff) == 0xA5)
			{
				count = serial_->read(&all_bytes[1], 1);
				if ((int)(all_bytes[1]&0xff) == 0x6A)
				{
					//all_bytes[0] = 0xA5;
					//all_bytes[1] = 0x6A;
					total = MAX_BUF - 2;
					Intercept = 2;
					i = 0;
					break;
				}
			}
			if(time > 8200) //lidar disconnected about 200ms
			{
				link_fail = true;
				first_data = true;
				time=0;
				break;
			}
			
			if(count <= 0) 
				time++;
			else
				time = 0;
		}
		first_data = false;
	}
	
	if(link_fail) 
	{
		link_fail = false;
		scan_health_ = -3;
		continue;
	}
	
	count = serial_->read(all_bytes+i+Intercept, total-i);
	if(time > 8200) //lidar disconnected about 200ms
	{
		time=0;
		first_data = true;
		scan_health_ = -3;
		continue;
	}
	time++;	

	if (count > 0){
		time=0;
		i += count;

		//Read data until the entire array is full
		if (i >= total){
			//Find the header of the next data package to calculate the total length of the current data package
			for(int k=1; k<MAX_BUF - 1; k++)
			{
				if((int)(all_bytes[k]&0xff) == 0xA5 && ((int)(all_bytes[k+1]&0xff) == 0x5A || (int)(all_bytes[k+1]&0xff) == 0x6A))
				{
					total = k;
					break;
				}
			}

			rpm_ = ((all_bytes[2]&0x7f) << 8) + (all_bytes[3]&0xff);
			int flag = ((all_bytes[2] & 0x80) == 0) ? 0 : 1;
			int angular_resolution = (all_bytes[4]&0xff) >> 1;
			int start_angle = ((all_bytes[4] & 0x01) << 8) + (all_bytes[5]&0xff);
			
			if(start_angle < last_angle)
			{
			  t2 = boost::posix_time::microsec_clock::universal_time();
			  boost::posix_time::millisec_posix_time_system_config::time_duration_type t_elapse;
			  t_elapse = t2 - t1;
			  real_rpm_ = 1000000.0 / t_elapse.ticks();
			  t1 = t2;

			  boost::unique_lock<boost::mutex> lock(mutex_);
			  scan_points_bak_.resize(scan_points_.size());
			  scan_points_bak_.assign(scan_points_.begin(), scan_points_.end());
			  pre_time_ = time_;

			  lock.unlock();
			  pubscan_cond_.notify_one();
			  time_ = ros::Time::now();
			}
			last_angle = start_angle;
			
			/*if (angular_resolution/100.0 != resolution_)
			{
			  ROS_WARN("angular resolution is not ok yet current res %d", angular_resolution);
			  usleep(100000);

				time=0;
				first_data = true;
				scan_health_ = -3;
				continue;
			}*/

			int len = total-6;
			if(len < 0) 
			{
				len = 0;
			}
			if(len > 180) 
			{
				len = 180;
			}
			memset(packet_bytes, 0, data_len_);
			memcpy(packet_bytes, all_bytes+6, len);
			
			
			for (int i = 0; i < len; i = i+3)
			{
			  int idx = start_angle / resolution_ + i/3;
			  if(idx > 1439) 
			  {
				  continue;
			  }
			  
			  double distance = ((packet_bytes[i+1] & 0xFF) << 8) | (packet_bytes[i+2] & 0xFF);         
			  double degree = idx * resolution_;

			  boost::unique_lock<boost::mutex> lock(mutex_);
			  scan_points_[idx].degree = degree;
			  scan_points_[idx].range = distance/1000.0;
			  if(use_angle_)
			  {
				scan_points_[idx].intensity = 0;
			  }
			  else
			  {
				scan_points_[idx].intensity = packet_bytes[i] & 0xFF;
			  }
			  lock.unlock();
			}
			
			Intercept = MAX_BUF - total;

			//Remove the parsed data part from the array
			memcpy(all_bytes, all_bytes+total, Intercept);
			i = 0;
		}
	}
  }
  if (packet_bytes)
  {
    packet_bytes = NULL;
    delete packet_bytes;
  }
  if (all_bytes)
  {
    all_bytes = NULL;
    delete all_bytes;
  }
}

double LS01B::angle_compensate(double dist)
{
  double offset = (0.02345 / dist) * 57.3;
  return offset;
}

void LS01B::pubScanThread()
{
  bool wait_for_wake = true;
  boost::unique_lock<boost::mutex> lock(pubscan_mutex_);

  while (ros::ok() && !is_shutdown_)
  {
    while (wait_for_wake)
    {
      //ROS_WARN("pubScanThread thread is suspending");
      pubscan_cond_.wait(lock);
      wait_for_wake = false;
    }

    std::vector<ScanPoint> points;
    ros::Time start_time;
    float scan_time;
    this->getScan(points, start_time, scan_time);
    int count = points.size();
    if (count <= 0)
      continue;

    sensor_msgs::LaserScan msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = start_time;
    msg.angle_min = 0.0;
    msg.angle_max = 2 * M_PI;
    msg.angle_increment = (msg.angle_max - msg.angle_min) / count;
    msg.range_min = 0.15;
    msg.range_max = 16;
    msg.ranges.resize(count);
    msg.intensities.resize(count);
    msg.scan_time = scan_time;
    msg.time_increment = scan_time / (double)(count - 1);

    for (int i = count - 1; i >= 0; i--) {
      if (points[count - i - 1].range == 0.0) {
        msg.ranges[i] = std::numeric_limits<float>::infinity();
        msg.intensities[i] = 0;
      }
      else {
        double dist = points[count - i - 1].range;
        if (flag_angle_compensate) {
          double offset = angle_compensate(dist);
          int actual_index = i;
          int fix_index = int16_t (round(offset / resolution_));
            ROS_DEBUG("dist is %2.2f, current idx is %d, offset is %2.3f, fix_index is %d",dist, i,  offset, fix_index);

          int j = (i - fix_index);
            if (j > 0) {
              actual_index = j;
            } else {
              actual_index = points_size_ + j;
            }
            msg.ranges[actual_index] = (float) dist;
            if (i == 0)
            {
                msg.ranges[i] = (msg.ranges[1] + msg.ranges[count-1]) / 2.0;
            }
        } else{
          msg.ranges[i] = (float) dist;
        }
        msg.intensities[i] = points[count - i - 1].intensity;
      }

      if ((truncated_mode_ == 1) or (truncated_mode_ == 3))
      {
        for (int j = 0; j < disable_angle_max_range_.size(); ++j) {
          if ((i >= (disable_angle_min_range_[j] * count / 360) ) &&
              (i <= (disable_angle_max_range_[j] * count / 360 ))) {
            msg.ranges[i] = special_range_;
            msg.intensities[i] = 0;
          }
        }
      }
      if ((truncated_mode_ == 2) or (truncated_mode_ == 3))
      {
        double point_dist = msg.ranges[i];
        if (point_dist < 1.0 && point_dist > 0.06)
        {
          double x = point_dist * cos(i * resolution_ * M_PI / 180);
          double y = point_dist * sin(i * resolution_ * M_PI / 180);

          double dist2center = sqrt((y - center_y_) * (y - center_y_) + (x - center_x_) * (x - center_x_));
          if (dist2center < robot_radius_)
          {
            msg.ranges[i] = special_range_;
            msg.intensities[i] = 0;
          }
        }
      }
    }
    pub_.publish(msg);
    // ROS_INFO("getRPM = %f", getRPM());
    wait_for_wake = true;
  }
}

void LS01B::run()
{
  setResolution(resolution_);
  usleep(100000);
  setMotorSpeed(rpm_);
  usleep(100000);
  switchData(false);
  usleep(100000);
  startScan();
  usleep(100000);
  startScan();
  usleep(100000);
  setScanMode(true); 
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
  ros::init(argv, argc, "ls01b");
 
  ls::LS01B* ls01b = ls::LS01B::instance();
  usleep(100000);
  ls01b->run();
  ros::spin();
  return 0;
}
