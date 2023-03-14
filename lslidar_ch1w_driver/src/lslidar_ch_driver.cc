#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <lslidar_ch_driver/lslidar_ch_driver.h>

namespace lslidar_ch_driver {

    LslidarChDriver::LslidarChDriver(
            ros::NodeHandle &n, ros::NodeHandle &pn) :
            nh(n),
            pnh(pn),
            socket_id(-1) {
        return;
    }

    LslidarChDriver::~LslidarChDriver() {

        if (nullptr == difop_thread_) {
//            difop_thread_->interrupt();
            difop_thread_->join();
        }
        return;
    }

    bool LslidarChDriver::loadParameters() {
        pnh.param("frame_id", frame_id, std::string("lslidar"));
        pnh.param("lidar_ip", lidar_ip_string, std::string("192.168.1.200"));
        pnh.param<int>("device_port", UDP_PORT_NUMBER, 2368);
        pnh.param<bool>("add_multicast", add_multicast, false);
        pnh.param("group_ip", group_ip_string, std::string("234.2.3.2"));
        pnh.param("msop_port", msop_udp_port, (int) MSOP_DATA_PORT_NUMBER);
        pnh.param("difop_port", difop_udp_port, (int) DIFOP_DATA_PORT_NUMBER);
        pnh.param("use_time_service", use_time_service, false);
        pnh.param("pcap", dump_file, std::string(""));

        inet_aton(lidar_ip_string.c_str(), &lidar_ip);
        // ROS_INFO_STREAM("Opening UDP socket: address " << lidar_ip_string);
        if (add_multicast) ROS_INFO_STREAM("Opening UDP socket: group_address " << group_ip_string);
        // ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
        ROS_INFO_STREAM("using time service or not: " << use_time_service);

        return true;
    }

    bool LslidarChDriver::createRosIO() {

        packet_pub = nh.advertise<lslidar_ch1w_msgs::LslidarChPacket>(
                "lslidar_packet_1w", 100);
        double packet_rate = 5507;
        if (dump_file != "") {
            msop_input_.reset(new lslidar_ch_driver::InputPCAP(pnh, msop_udp_port, packet_rate, dump_file));
            difop_input_.reset(new lslidar_ch_driver::InputPCAP(pnh, difop_udp_port, 1, dump_file));
        } else {
            msop_input_.reset(new lslidar_ch_driver::InputSocket(pnh, msop_udp_port));
            difop_input_.reset(new lslidar_ch_driver::InputSocket(pnh, difop_udp_port));
        }
        difop_thread_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&LslidarChDriver::difopPoll, this)));

        return true;
    }

    void LslidarChDriver::initTimeStamp(void) {

        for (int i = 0; i < 10; ++i) {
            this->packetTimeStamp[i] = 0;
        }
        for (int j = 0; j < 4; ++j) {
            prism_angle[j] = 0.0;
        }
        this->prism_offset = 0.0;
        this->prism_flag = true;
        this->pointcloudTimeStamp = 0;
        this->timeStamp = ros::Time(0.0);
    }

    bool LslidarChDriver::initialize() {

        this->initTimeStamp();

        if (!loadParameters()) {
            ROS_ERROR("Cannot load all required ROS parameters...");
            return false;
        }

        if (!createRosIO()) {
            ROS_ERROR("Cannot create all ROS IO...");
            return false;
        }
        return true;
    }

    bool LslidarChDriver::polling() {
        // Allocate a new shared pointer for zero-copy sharing with other nodelets.
        lslidar_ch1w_msgs::LslidarChPacketPtr packet(
                new lslidar_ch1w_msgs::LslidarChPacket());

        // Since the lslidar delivers data at a very high rate, keep
        // reading and publishing scans as fast as possible.
        while (true) {
            // keep reading until full packet received
            int rc = msop_input_->getPacket(packet);
            if (rc == 0) break;       // got a full packet?
            if (rc < 0) return false; // end of file reached?
        }

        if (use_time_service) {
            // it is already the msop msg
            lslidar_ch1w_msgs::LslidarChPacket pkt = *packet;

            if(pkt.data[1205] == 0x01){
                this->packetTimeStamp[4] = pkt.data[1199];
                this->packetTimeStamp[5] = pkt.data[1198];
                this->packetTimeStamp[6] = pkt.data[1197];
            }else if(pkt.data[1205] == 0x02){
                this->packetTimeStamp[4] = pkt.data[1199];
            }

            cur_time.tm_sec = this->packetTimeStamp[4];
            cur_time.tm_min = this->packetTimeStamp[5];
            cur_time.tm_hour = this->packetTimeStamp[6];
            cur_time.tm_mday = this->packetTimeStamp[7];
            cur_time.tm_mon = this->packetTimeStamp[8] - 1;
            cur_time.tm_year = this->packetTimeStamp[9] + 2000 - 1900;
            this->pointcloudTimeStamp = static_cast<uint64_t>(timegm(&cur_time));
            uint64_t packet_timestamp;
            packet_timestamp = (pkt.data[1203] +
                                pkt.data[1202] * pow(2, 8) +
                                pkt.data[1201] * pow(2, 16) +
                                pkt.data[1200] * pow(2, 24)) * 1e3; //ns

            timeStamp = ros::Time(this->pointcloudTimeStamp, packet_timestamp);// s,ns
            packet->stamp = timeStamp;
        } else {
            packet->stamp = ros::Time::now();
        }
        for (int i = 0; i < 4; ++i) {
            packet->prism_angle[i] = prism_angle[i];
        }
        packet->prism_offset_angle = prism_offset;
        packet_pub.publish(*packet);
        return true;
    }


    void LslidarChDriver::difopPoll(void) {
        lslidar_ch1w_msgs::LslidarChPacketPtr difop_packet(
                new lslidar_ch1w_msgs::LslidarChPacket());

        // reading and publishing scans as fast as possible.
        while (ros::ok()) {
            // keep reading
            int rc = difop_input_->getPacket(difop_packet);
            if (rc == 0) {
                if (difop_packet->data[0] == 0xa5 && difop_packet->data[1] == 0xff && difop_packet->data[2] == 0x00 &&
                    difop_packet->data[3] == 0x5a) {

                    if(difop_packet->data[176] == 0x00){
                        this->packetTimeStamp[7] = difop_packet->data[54];
                        this->packetTimeStamp[8] = difop_packet->data[53];
                        this->packetTimeStamp[9] = difop_packet->data[52];
                    }else if(difop_packet->data[176] == 0x01){
                        this->packetTimeStamp[5] = difop_packet->data[56];
                        this->packetTimeStamp[6] = difop_packet->data[55];
                        this->packetTimeStamp[7] = difop_packet->data[54];
                        this->packetTimeStamp[8] = difop_packet->data[53];
                        this->packetTimeStamp[9] = difop_packet->data[52];
                    }
                    if(prism_flag){
                        int angle0 = difop_packet->data[240] * 256 + difop_packet->data[241];
                        angle0 = angle0 > 32767 ? (angle0 - 65536) : angle0;
                        this->prism_offset = angle0 * 0.01;

                        int angle1 = difop_packet->data[242] * 256 + difop_packet->data[243];
                        angle1 = angle1 > 32767 ? (angle1 - 65536) : angle1;
                        this->prism_angle[0] = angle1 * 0.01;

                        int angle2 = difop_packet->data[244] * 256 + difop_packet->data[245];
                        angle2 = angle2 > 32767 ? (angle2 - 65536) : angle2;
                        this->prism_angle[1] = angle2 * 0.01;

                        int angle3 = difop_packet->data[246] * 256 + difop_packet->data[247];
                        angle3 = angle3 > 32767 ? (angle3 - 65536) : angle3;
                        this->prism_angle[2] = angle3 * 0.01;

                        int angle4 = difop_packet->data[248] * 256 + difop_packet->data[249];
                        angle4 = angle4 > 32767 ? (angle4 - 65536) : angle4;
                        this->prism_angle[3] = angle4 * 0.01;
                        prism_flag = false;
                    }
                }
                //getFPGA_GPSTimeStamp(difop_packet);
            } else if (rc < 0) {
                return;
            }
            ros::spinOnce();
        }
    }
} // namespace lslidar_driver
