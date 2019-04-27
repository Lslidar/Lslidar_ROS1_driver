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

#include <lslidar_c16_compensator/compensator.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace lslidar_c16_compensator {

class CompensatorNodelet : public nodelet::Nodelet {
 public:
  CompensatorNodelet() {}
  ~CompensatorNodelet() {}

 private:
  virtual void onInit();
  boost::shared_ptr<Compensator> compensator_;
};

/** @brief Nodelet initialization. */
void CompensatorNodelet::onInit() {
  ROS_INFO("Compensator nodelet init");
  compensator_.reset(new Compensator(getNodeHandle(), getPrivateNodeHandle()));
}

} // end namespace lslidar_c16_decoder

PLUGINLIB_DECLARE_CLASS(lslidar_c16_compensator, CompensatorNodelet,
    lslidar_c16_compensator::CompensatorNodelet, nodelet::Nodelet);
