#include <lslidar_ch_decoder/lslidar_ch_decoder_nodelet.h>

namespace lslidar_ch_decoder {

void LslidarChDecoderNodelet::onInit() {
  decoder.reset(new LslidarChDecoder(
        getNodeHandle(), getPrivateNodeHandle()));
  if(!decoder->initialize()) {
    ROS_ERROR("Cannot initialize the lslidar puck decoder...");
    return;
  }
  return;
}

} // end namespace lslidar_ch_decoder

PLUGINLIB_EXPORT_CLASS(
    lslidar_ch_decoder::LslidarChDecoderNodelet, nodelet::Nodelet);
