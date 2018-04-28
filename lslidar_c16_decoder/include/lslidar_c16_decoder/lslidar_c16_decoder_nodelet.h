/*
 * This file is part of lslidar_c16 driver.
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

#ifndef LSLIDAR_C16_DECODER_NODELET_H
#define LSLIDAR_C16_DECODER_NODELET_H

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <lslidar_c16_decoder/lslidar_c16_decoder.h>

namespace lslidar_c16_decoder {
class LslidarC16DecoderNodelet: public nodelet::Nodelet {
public:

  LslidarC16DecoderNodelet() {}
  ~LslidarC16DecoderNodelet() {}

private:

  virtual void onInit();
  LslidarC16DecoderPtr decoder;
};

} // end namespace lslidar_n301_decoder


#endif
