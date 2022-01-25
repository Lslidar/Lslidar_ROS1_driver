/*******************************************************
@company: Copyright (C) 2018, Leishen Intelligent System
@product: LS01B
@filename: ls01b.h
@brief:
@version:       date:       author:     comments:
@v1.0           18-8-21     fu          new
@v1.5           19-04-18     tongsky    Fix bug when switching resolution
*******************************************************/
#ifndef LS01B_H
#define LS01B_H
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "ls01b_v2/lsiosr.h"
#include "ls01b_v2/resolution.h"
#include <std_srvs/SetBool.h>
#include <dynamic_reconfigure/server.h>
#include "ls01b_v2/FilterConfig.h"

namespace ls {

typedef struct {
    double degree;
    double range;
    double intensity;
} ScanPoint;

//typedef struct {
//  std::chrono::steady_clock::time_point time_stamp;
//  std::vector<ScanPoint> points;
//}ScanMsg;

class LS01B
{
public:
  LS01B();
  ~LS01B();
    /**
    * 实例化雷达
    * port: 串口号，
    * baud_rate: 波特率 460800
    * resolution: 雷达角度分辨率1.0度 0.5度 0.25度
    */
    static LS01B* instance();

    /**
    * 判断雷达是否有效
    */
    bool isHealth();

    /**
     * 恢复雷达状态
     */
    bool resetHealth();

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

    /**
     * start lidar, System work normally
     */
    int startScan();

    /**
     * stop lidar, System in sleep mode
     */
    int stopScan();

    /**
     * set scan mode. continouse or once
     * is_continuous:
     */
    int setScanMode(bool is_continuous);

    /**
     * stop recv data from lidar
     */
    int stopRecvData();
	
    /**
     * switch angle or intensity in recv data
     * use_angle: true. the angle is chosed
     *            false. the intensity is chosed
     */
    int switchData(bool use_angle);

    /**
     * set the motor spped
     * rpm: round per minute
     */
    int setMotorSpeed(int rpm);

    /**
    * 设置雷达数据分辨率
    * resolution: 分辨率
    */
    int setResolution(double resolution);

    double getRPM();

    void run();

    double angle_compensate(double dist);

    bool resServerCallback(ls01b_v2::resolution::Request &req,
                   ls01b_v2::resolution::Response &res);

    bool startCallback(std_srvs::SetBoolRequest &req,
                   std_srvs::SetBoolResponse &res);

  private:
    void initParam();
    void recvThread();
    void pubScanThread();
    void DynParamCallback(ls01b_v2::FilterConfig &config, uint32_t level);

    uint16_t checkSum(const uint8_t *p_byte);

    std::vector<ScanPoint> scan_points_;
    std::vector<ScanPoint> scan_points_bak_;
//    ScanMsg scan_msg_;

    LSIOSR * serial_;
    boost::thread *recv_thread_;
    boost::thread *pubscan_thread_;
    boost::mutex pubscan_mutex_;
    boost::mutex mutex_;
    boost::condition_variable pubscan_cond_;
    ros::ServiceServer changeRes_client;
    ros::ServiceServer start_client;

    bool is_shutdown_;    // shutdown recvthread
    bool is_start_;       // begin to read data from lidar
    bool use_angle_;
    bool start_switch;
    bool flag_angle_compensate;
    
    int scan_health_; // 0 OK
    double resolution_;
    int data_len_;

    int points_size_;
    int rpm_;
    double real_rpm_;
    double special_range_;

    ros::Time pre_time_;
    ros::Time time_;

    std::string serial_port_;
    int baud_rate_;
    std::string scan_topic_;
    std::string frame_id_;

    double robot_radius_;
    int truncated_mode_;
    std::vector<int> disable_angle_max_range_;
    std::vector<int> disable_angle_min_range_;
    double center_x_;
    double center_y_;

    double rmp_;
    double angle_disable_min_0;
    double angle_disable_max_0;
    double angle_disable_min_1;
    double angle_disable_max_1;
    double angle_disable_min_2;
    double angle_disable_max_2;
    double angle_disable_min_3;
    double angle_disable_max_3;
    double angle_disable_min_4;
    double angle_disable_max_4;

    ros::NodeHandle n_;
    ros::Publisher pub_;
};

}
#endif
