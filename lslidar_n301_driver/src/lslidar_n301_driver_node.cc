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

#include <ros/ros.h>
#include <lslidar_n301_driver/lslidar_n301_driver.h>

volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lslidar_n301_driver_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");
	
	signal(SIGINT, my_handler);
	
    // start the driver
    lslidar_n301_driver::LslidarN301Driver driver(node, private_nh);
  if (!driver.initialize()) {
    ROS_ERROR("Cannot initialize lslidar driver...");
    return 0;
  }
    // loop until shut down or end of file
    while(ros::ok() && driver.polling()) {
        ros::spinOnce();

    }

    return 0;
}
