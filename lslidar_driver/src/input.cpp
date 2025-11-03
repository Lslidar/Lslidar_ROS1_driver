/******************************************************************************
 * This file is part of lslidar_driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "lslidar_driver/input.hpp"

extern volatile sig_atomic_t flag;
namespace lslidar_driver {

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
    Input::Input(ros::NodeHandle private_nh, uint16_t port, int packet_size) : private_nh_(private_nh), port_(port),
                                                                               packet_size_(packet_size) {
        npkt_update_flag_ = false;
        cur_rpm_ = 0;
        return_mode_ = 1;

        private_nh.param("device_ip", devip_str_, std::string(""));
        private_nh.param<bool>("add_multicast", add_multicast, false);
        private_nh.param<std::string>("group_ip", group_ip, "224.1.1.2");
        private_nh.param<int>("difop_port", difop_port_, 2369);
        // if (!devip_str_.empty())
        //     ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);
    }


////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
    InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port, int packet_size) : Input(private_nh, port,
                                                                                                 packet_size) {
        sockfd_ = -1;

        if (!devip_str_.empty()) {
            inet_aton(devip_str_.c_str(), &devip_);
        }
        
        ROS_INFO_STREAM("Opening UDP socket port: " << port);
        sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
        if (sockfd_ == -1) {
            perror("socket");  // TODO: ROS_ERROR errno
            return;
        }

        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *) &opt, sizeof(opt))) {
            perror("setsockopt error!\n");
            return;
        }

        sockaddr_in my_addr;                   // my address information
        memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
        my_addr.sin_family = AF_INET;          // host byte order
        my_addr.sin_port = htons(port);        // port in network byte order
        my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

        if (bind(sockfd_, (sockaddr *) &my_addr, sizeof(sockaddr)) == -1) {
            perror("bind");  // TODO: ROS_ERROR errno
            return;
        }

        if (add_multicast) {
            struct ip_mreq group;
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
            group.imr_interface.s_addr = htonl(INADDR_ANY);
            //group.imr_interface.s_addr = inet_addr("192.168.1.102");

            if (setsockopt(sockfd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &group, sizeof(group)) < 0) {
                perror("Adding multicast group error ");
                close(sockfd_);
                exit(1);
            } else
                printf("Adding multicast group...OK.\n");
        }
        if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
            perror("non-block");
            return;
        }
    }


/** @brief destructor */
    InputSocket::~InputSocket(void) {
        (void) close(sockfd_);
    }

/** @brief Get one lslidar packet. */
    int InputSocket::getPacket(lslidar_msgs::LslidarPacketPtr &pkt) {
        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN; 
        static const int POLL_TIMEOUT = 2000;  // 超时时间 (ms)

        sockaddr_in sender_address{};
        socklen_t sender_address_len = sizeof(sender_address);

        int retval = poll(fds, 1, POLL_TIMEOUT);

        if (retval > 0 && (fds[0].revents & POLLIN)) {
            ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], packet_size_, 0, (sockaddr *) &sender_address, &sender_address_len);
            
            if (sender_address.sin_addr.s_addr == devip_.s_addr) {
                return nbytes;
            } else {
                // IP 地址不一致
                ROS_WARN_THROTTLE(2, "lidar IP parameter mismatch. Received IP: %s. Please reset lidar IP in the launch file.", inet_ntoa(sender_address.sin_addr));
                return 0;
            }
        } else {
            if (retval == 0) {  
                time_t curTime = time(NULL);
                struct tm *curTm = localtime(&curTime);
                char bufTime[72] = {0};
                sprintf(bufTime, "%d-%d-%d %d:%d:%d", curTm->tm_year + 1900, curTm->tm_mon + 1,
                        curTm->tm_mday, curTm->tm_hour, curTm->tm_min, curTm->tm_sec);
                ROS_WARN("%s  lslidar poll() timeout, port:%d", bufTime, port_);
                return 0; // 超时返回
            }

            if (retval < 0) { 
                if (errno != EINTR) {
                    ROS_ERROR("poll() error: %s", strerror(errno));
                }
                return -1; 
            }

            if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) {
                ROS_ERROR("poll() reports lslidar error");
                return -1;
            }
        }

        return 0;
    }

    void InputSocket::sendPacket(const unsigned char *data, size_t length) {
        if (data == nullptr || length <= 0) {
            ROS_ERROR("Invalid input data or length.");
            return;
        }

        sockaddr_in server_sai;
        server_sai.sin_family = AF_INET;
        server_sai.sin_port = htons(difop_port_);
        server_sai.sin_addr.s_addr = inet_addr(devip_str_.c_str());

        ssize_t nbytes = sendto(sockfd_, data, length, 0, (struct sockaddr *)&server_sai, sizeof(server_sai));

        if (nbytes < 0) {
            ROS_ERROR("Data packet sending failed: %s", strerror(errno));
        } else if (nbytes != length) {
            ROS_WARN("Partial data sent: %zd/%zd bytes", nbytes, length);
        } else {
            ROS_INFO("Successfully sent %zd bytes!", nbytes);
        }
    }


////////////////////////////////////////////////////////////////////////
// InputPCAP class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
    InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, int packet_size, double packet_rate,
                         std::string filename,
                         bool read_once, bool read_fast, double repeat_delay)
            : Input(private_nh, port, packet_size), packet_rate_(packet_rate), filename_(filename) {
        pcap_ = NULL;
        empty_ = true;

        // get parameters using private node handle
        private_nh.param("read_once", read_once_, false);
        private_nh.param("read_fast", read_fast_, false);
        private_nh.param("repeat_delay", repeat_delay_, 0.0);

        if (read_once_)
            ROS_INFO("Read input file only once.");
        if (read_fast_)
            ROS_INFO("Read input file as quickly as possible.");
        if (repeat_delay_ > 0.0)
            ROS_INFO("Delay %.3f seconds before repeating input file.", repeat_delay_);

        // Open the PCAP dump file
        // ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
        ROS_INFO_STREAM("Opening PCAP file " << filename_);
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL) {
            ROS_FATAL("Error opening lslidar socket dump file.");
            return;
        }

        std::stringstream filter;
        if (devip_str_ != "")  // using specific IP?
        {
            filter << "src host " << devip_str_ << " && ";
        }
        filter << "udp dst port " << port;
        pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
    }

/** destructor */
    InputPCAP::~InputPCAP(void) {
        pcap_close(pcap_);
    }

/** @brief Get one lslidar packet. */
    int InputPCAP::getPacket(lslidar_msgs::LslidarPacketPtr &pkt) {
        struct pcap_pkthdr *header;
        const u_char *pkt_data;

        while (flag == 1) {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
                // Skip packets not for the correct port and from the
                // selected IP address.
                if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
                    continue;

                // Keep the reader from blowing through the file.
                if (read_fast_ == false)
                    packet_rate_.sleep();


                memcpy(&pkt->data[0], pkt_data + 42, packet_size_);

                if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 &&
                    pkt->data[3] == 0x5A) {//difop
                    int rpm = (pkt->data[8] << 8) | pkt->data[9];
                    ROS_DEBUG("lidar rpm:%d", rpm);
                }

                pkt->stamp = ros::Time::now();  // time_offset not considered here, as no
                // synchronization required
                empty_ = false;
                return packet_size_;  // success
            }

            if (empty_)  // no data in file?
            {
                ROS_WARN("Error %d reading lslidar packet: %s", res, pcap_geterr(pcap_));
                return -1;
            }

            if (read_once_) {
                ROS_INFO("end of file reached -- done reading.");
                return -1;
            }

            if (repeat_delay_ > 0.0) {
                ROS_INFO("end of file reached -- delaying %.3f seconds.", repeat_delay_);
                usleep(rint(repeat_delay_ * 1000000.0));
            }

            ROS_INFO("Replaying lslidar dump file");

            // I can't figure out how to rewind the file, because it
            // starts with some kind of header.  So, close the file
            // and reopen it with pcap.
            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;  // maybe the file disappeared?
        }                 // loop back and try again

        if (flag == 0) {
            abort();
        }

        return 0;
    }

    void InputPCAP::sendPacket(const unsigned char *data, size_t length) {
        ROS_INFO("Offline settings are not currently supported.");
    }

} //namespace
