
#include "lslidar_ch_driver/input.h"

extern volatile sig_atomic_t flag;
namespace lslidar_ch_driver {
    static const size_t packet_size = sizeof(lslidar_ch1w_msgs::LslidarChPacket().data);
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
    Input::Input(ros::NodeHandle private_nh, uint16_t port) : private_nh_(private_nh), port_(port) {

        private_nh.param("lidar_ip", devip_str_, std::string(""));
        private_nh.param<bool>("add_multicast", add_multicast, false);
        private_nh.param<std::string>("group_ip", group_ip, "224.1.1.2");
        if (!devip_str_.empty())
            ROS_INFO_STREAM("Only accepting packets from IP address: " << devip_str_);
    }


////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
    InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port) : Input(private_nh, port) {
        sockfd_ = -1;

        if (!devip_str_.empty()) {
            inet_aton(devip_str_.c_str(), &devip_);
        }

        ROS_INFO_STREAM("Opening UDP socket: port " << port);
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
            perror("bind");
            return;
        }
        if (add_multicast) {
            struct ip_mreq group;
            group.imr_multiaddr.s_addr = inet_addr(group_ip.c_str());
            group.imr_interface.s_addr = htonl(INADDR_ANY);
            if(setsockopt(sockfd_,IPPROTO_IP,IP_ADD_MEMBERSHIP,(char*)&group, sizeof(group)) < 0){
                perror("Adding multicast group error");
                close(sockfd_);
                exit(1);
            }else{
                printf("Adding multicast group ok.\n");
            }
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

    int InputSocket::getPacket(lslidar_ch1w_msgs::LslidarChPacketPtr &packet) {

        struct pollfd fds[1];
        fds[0].fd = sockfd_;
        fds[0].events = POLLIN;
        static const int POLL_TIMEOUT = 2000; // one second (in msec)

        sockaddr_in sender_address;
        socklen_t sender_address_len = sizeof(sender_address);
        //while (true)
        while (flag == 1) {
            // poll() until input available
            do {
                int retval = poll(fds, 1, POLL_TIMEOUT);
                if (retval < 0)             // poll() error?
                {
                    if (errno != EINTR)
                        ROS_ERROR("poll() error: %s", strerror(errno));
                    return 1;
                }
                if (retval == 0)            // poll() timeout?
                {
                    ROS_WARN("lslidar poll() timeout");
                    return 1;
                }
                if ((fds[0].revents & POLLERR)
                    || (fds[0].revents & POLLHUP)
                    || (fds[0].revents & POLLNVAL)) // device error?
                {
                    ROS_ERROR("poll() reports lslidar error");
                    return 1;
                }
            } while ((fds[0].revents & POLLIN) == 0);

            // Receive packets that should now be available from the
            // socket using a blocking read.
            ssize_t nbytes = recvfrom(sockfd_, &packet->data[0], PACKET_SIZE, 0,
                                      (sockaddr *) &sender_address, &sender_address_len);

            //        ROS_DEBUG_STREAM("incomplete lslidar packet read: "
            //                         << nbytes << " bytes");

            if (nbytes < 0) {
                if (errno != EWOULDBLOCK) {
                    perror("recvfail");
                    ROS_INFO("recvfail");
                    return 1;
                }
            } else if ((size_t) nbytes == PACKET_SIZE) {

                // read successful,
                // if packet is not from the lidar scanner we selected by IP,
                // continue otherwise we are done
                if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr) {
                    ROS_WARN_THROTTLE(2, "launch file param lidar ip set error");
                    continue;
                } else
                    break; //done
            }

        }
        if (flag == 0) {
            abort();
        }

        return 0;
    }

    InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
                         bool read_once, bool read_fast, double repeat_delay) : Input(private_nh, port),
                                                                                packet_rate_(packet_rate),
                                                                                filename_(filename) {
        pcap_ = nullptr;
        empty_ = true;
        private_nh.param("read_once", read_once_, false);
        private_nh.param("read_fast", read_fast_, false);
        private_nh.param("repeat_delay", repeat_delay_, 0.0);
        ROS_INFO_STREAM("opening pcap file: " << filename_);
        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == nullptr) {
            ROS_FATAL("Error opening lslidar socket dump file.");
            return;
        }
        std::stringstream filter;
        if (devip_str_ != "") {
            filter << "src host " << devip_str_ << "&&";
        }
        filter << "udp dst port " << port;
        pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
    }


    InputPCAP::~InputPCAP() {
        pcap_close(pcap_);
    }


    int InputPCAP::getPacket(lslidar_ch1w_msgs::LslidarChPacketPtr &packet) {
        struct pcap_pkthdr *header;
        const u_char *pkt_data;
        while (flag == 1) {
            int res;
            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0) {
                if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data))) {
                    continue;
                }
                if (read_fast_ == false) {
                    packet_rate_.sleep();
                }
                mempcpy(&packet->data[0], pkt_data + 42, packet_size);
                empty_ = false;
                return 0;
            }
            if (empty_) {
                ROS_WARN("Error %d reading lslidar packet: %s", res, pcap_geterr(pcap_));
            }
            if (read_once_) {
                ROS_INFO("end of file reached-- done reading.");
            }
            if (repeat_delay_ > 0.0) {
                ROS_INFO("end of file reached-- delaying %.3f seconds.", repeat_delay_);
                usleep(rint(repeat_delay_ * 1000000.0));
            }
            ROS_DEBUG("replaying lslidar dump file");
            pcap_close(pcap_);
            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
            empty_ = true;

        }
        if (flag == 0) {
            abort();
        }
    }


}  //namespace
