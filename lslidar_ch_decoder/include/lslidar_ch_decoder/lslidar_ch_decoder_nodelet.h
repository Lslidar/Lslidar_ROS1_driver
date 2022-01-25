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

#ifndef LSLIDAR_Ch_DECODER_NODELET_H
#define LSLIDAR_Ch_DECODER_NODELET_H

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <lslidar_ch_decoder/lslidar_ch_decoder.h>

namespace lslidar_ch_decoder {
class LslidarChDecoderNodelet: public nodelet::Nodelet {
public:

  LslidarChDecoderNodelet() {}
  ~LslidarChDecoderNodelet() {}

private:

  virtual void onInit();
  LslidarChDecoderPtr decoder;
};

} // end namespace lslidar_ch_decoder


#endif
