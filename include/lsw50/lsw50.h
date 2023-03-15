/*******************************************************
@company: Copyright (C) 2021, Leishen Intelligent System
@product: LSW50
@filename: lsw50.cpp
@brief:
@version:       date:       author:     comments:
@v1.0           21-2-4      yao          new
*******************************************************/
#ifndef LSW50_H
#define LSW50_H
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "lsw50/lsiosr.h"

namespace ls {

typedef struct {
    double degree;
    double range;
} ScanPoint;

class LSW50
{
public:
  LSW50();
  ~LSW50();
    /**
    * 实例化雷达
    * port: 串口号，
    * baud_rate: 波特率 460800
    */
    static LSW50* instance();

    /**
    * 获取雷达数据
    * poins: 雷达点的数据。类型为ScanPoint数组
    */
    int getScan(std::vector<ScanPoint> &points, ros::Time &scan_time, float &scan_duration);

    /**
    * 获取软件版本号
    * version: 返回版本号
    */
    int getVersion(std::string &version);

    int getRate();
	
  private:
    void initParam();
    void recvThread();
    void pubScanThread();

    std::vector<ScanPoint> scan_points_;
    std::vector<ScanPoint> scan_points_bak_;

    LSIOSR * serial_;
    boost::thread *recv_thread_;
    boost::thread *pubscan_thread_;
    boost::mutex pubscan_mutex_;
    boost::mutex mutex_;
    boost::condition_variable pubscan_cond_;

    bool is_shutdown_;    // shutdown recvthread
    int baud_rate_testing;
    int data_len_;
    int points_size_;
	  int point_num;
	  double min_range;
    double max_range;
    double angle_disable_min;
    double angle_disable_max;
	  double angle_able_min;
    double angle_able_max;
    ros::Time pre_time_;
    ros::Time time_;
	  ros::Publisher device_pub;
	
    std::string serial_port_;
    int baud_rate_;
    std::string scan_topic_;
    std::string frame_id_;
	
    ros::NodeHandle n_;
    ros::Publisher pub_;
};

}
#endif
