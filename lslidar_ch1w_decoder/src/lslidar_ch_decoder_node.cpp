

#include <ros/ros.h>

#include <lslidar_ch_decoder/lslidar_ch_decoder.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "lslidar_ch1w_decoder_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    lslidar_ch_decoder::LslidarChDecoderPtr decoder(
                new lslidar_ch_decoder::LslidarChDecoder(nh, pnh));

    if (!decoder->initialize()) {
        ROS_INFO("Cannot initialize the decoder...");
        return -1;
    }

    ros::spin();

    return 0;
}
