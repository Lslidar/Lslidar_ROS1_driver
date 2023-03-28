#include <ros/ros.h>
#include "lslidar_driver/lslidar_driver.h"
#include "std_msgs/String.h"

using namespace lslidar_driver;
volatile sig_atomic_t flag = 1;

static void my_handler(int sig) {
    flag = 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lslidar_driver");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    signal(SIGINT, my_handler);
    private_nh.param("lidar_type", lidar_type, std::string("c16"));
    ROS_INFO("lslidar type: %s", lidar_type.c_str());

    // start the driver
    if ("c16" == lidar_type) {
        lslidar_driver::lslidarC16Driver dvr(node, private_nh);
        // loop until shut down or end of file
        if (!dvr.initialize()) {
            ROS_ERROR("cannot initialize lslidar driver.");
            return 0;
        }
        while (ros::ok() ) {
            dvr.poll();
            ros::spinOnce();
        }

    } else {
        lslidar_driver::lslidarC32Driver dvr(node, private_nh);
        // loop until shut down or end of file
        if (!dvr.initialize()) {
            ROS_ERROR("cannot initialize lslidar driver.");
            return 0;
        }
        while (ros::ok()) {
            dvr.poll();
            ros::spinOnce();
        }

    }
    return 0;
}
