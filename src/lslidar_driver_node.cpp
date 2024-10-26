#include "lslidar_driver/lslidar_driver.h"

using namespace lslidar_driver;

std::atomic<bool> running(true);

static void my_handler(int sig) {
    running.store(false);
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lslidar_driver");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    signal(SIGINT, my_handler);

    lslidar_driver::LslidarDriver dvr(node, private_nh);

    if (!dvr.initialize()) {
        ROS_ERROR("cannot initialize lslidar driver.");
        return 0;
    }

    std::unique_ptr<ThreadPool> threadPool = std::make_unique<ThreadPool>(1);

    threadPool->enqueue([&]() {
        while (running.load() && ros::ok()) {
            dvr.poll();
        }
    });

    ros::MultiThreadedSpinner spinner(4); // Use 4 threads
    spinner.spin();

    return 0;
}