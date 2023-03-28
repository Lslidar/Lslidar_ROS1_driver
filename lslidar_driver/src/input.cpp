#include "lslidar_driver/input.h"

extern volatile sig_atomic_t flag;
namespace lslidar_driver
{
////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
Input::Input(ros::NodeHandle private_nh, uint16_t port) : private_nh_(private_nh), port_(port)
{

  private_nh.param("device_ip", devip_str_, std::string(""));
  private_nh.param<bool>("add_multicast", add_multicast, false);
  private_nh.param<std::string>("group_ip", group_ip, "224.1.1.2");
  private_nh.param<int>("packet_size", packet_size_, 1206);
  if (!devip_str_.empty())
    ROS_INFO_STREAM_ONCE("Only accepting packets from IP address: " << devip_str_);
}


////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
*/
InputSocket::InputSocket(ros::NodeHandle private_nh, uint16_t port) : Input(private_nh, port)
{
  sockfd_ = -1;

  if (!devip_str_.empty())
  {
    inet_aton(devip_str_.c_str(), &devip_);
  }

  ROS_INFO_STREAM("Opening UDP socket: port " << port);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
  {
    perror("socket");  // TODO: ROS_ERROR errno
    return;
  }

  int opt = 1;
  if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
  {
    perror("setsockopt error!\n");
    return;
  }

  sockaddr_in my_addr;                   // my address information
  memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
  my_addr.sin_family = AF_INET;          // host byte order
  my_addr.sin_port = htons(port);        // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

  if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
  {
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
  if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
  {
    perror("non-block");
    return;
  }
}

/** @brief destructor */
InputSocket::~InputSocket(void)
{
  (void)close(sockfd_);
}

/** @brief Get one lslidar packet. */
int InputSocket::getPacket(lslidar_msgs::LslidarPacketPtr &pkt)
{
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  static const int POLL_TIMEOUT = 3000;  // one second (in msec)

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
  while (flag == 1)
 // while (true)
  {
    // Receive packets that should now be available from the
    // socket using a blocking read.
    // poll() until input available
    do
    {
      int retval = poll(fds, 1, POLL_TIMEOUT);
      if (retval < 0)  // poll() error?
      {
        if (errno != EINTR)
          ROS_ERROR("poll() error: %s", strerror(errno));
        return 1;
      }
      if (retval == 0)  // poll() timeout?
      {


        ROS_WARN("lslidar poll() timeout,port %d",port_ );
        /*
        char buffer_data[8] = "re-con";
        memset(&sender_address, 0, sender_address_len);          // initialize to zeros
        sender_address.sin_family = AF_INET;                     // host byte order
        sender_address.sin_port = htons(MSOP_DATA_PORT_NUMBER);  // port in network byte order, set any value
        sender_address.sin_addr.s_addr = devip_.s_addr;          // automatically fill in my IP
        sendto(sockfd_, &buffer_data, strlen(buffer_data), 0, (sockaddr*)&sender_address, sender_address_len);
		*/
        return 1;
      }
      if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
      {
        ROS_ERROR("poll() reports lslidar error");
        return 1;
      }
    } while ((fds[0].revents & POLLIN) == 0);
    ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], packet_size_, 0, (sockaddr*)&sender_address, &sender_address_len);
    if (nbytes < 0)
    {
      if (errno != EWOULDBLOCK)
      {
        perror("recvfail");
        ROS_INFO("recvfail");
        return 1;
      }
    }
    else if ((size_t)nbytes == packet_size_)
    {
      if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr){
          ROS_WARN_THROTTLE(2.0,"lidar IP parameter set error,please reset in launch file");
          continue;
      }
      else
        break;  // done
    }

    ROS_DEBUG_STREAM("incomplete lslidar packet read: " << nbytes << " bytes");
  }
  if (flag == 0)
  {
    abort();
  }

  return 0;
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
InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
                     bool read_once, bool read_fast, double repeat_delay)
  : Input(private_nh, port), packet_rate_(packet_rate), filename_(filename)
{
  pcap_ = nullptr;
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
  if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == nullptr)
  {
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
InputPCAP::~InputPCAP(void)
{
  pcap_close(pcap_);
}

/** @brief Get one lslidar packet. */
int InputPCAP::getPacket(lslidar_msgs::LslidarPacketPtr &pkt)
{
  struct pcap_pkthdr* header;
  const u_char* pkt_data;

 // while (flag == 1)
  while (flag == 1)
  {
    int res;
    if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
    {
      // Skip packets not for the correct port and from the
      // selected IP address.
      if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
        continue;

      // Keep the reader from blowing through the file.
      if (read_fast_ == false)
        packet_rate_.sleep();


      memcpy(&pkt->data[0], pkt_data + 42, packet_size_);

      if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A)
      {//difop
        int rpm = (pkt->data[8]<<8)|pkt->data[9];
        ROS_DEBUG("lidar rpm: %d",rpm);
      }
      empty_ = false;
      return 0;  // success
    }

    if (empty_)  // no data in file?
    {
      ROS_WARN("Error %d reading lslidar packet: %s", res, pcap_geterr(pcap_));
      return -1;
    }

    if (read_once_)
    {
      ROS_INFO("end of file reached -- done reading.");
      return -1;
    }

    if (repeat_delay_ > 0.0)
    {
      ROS_INFO("end of file reached -- delaying %.3f seconds.", repeat_delay_);
      usleep(rint(repeat_delay_ * 1000000.0));
    }

    ROS_DEBUG("replaying lslidar dump file");

    // I can't figure out how to rewind the file, because it
    // starts with some kind of header.  So, close the file
    // and reopen it with pcap.
    pcap_close(pcap_);
    pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
    empty_ = true;  // maybe the file disappeared?
  }                 // loop back and try again

  if (flag == 0)
  {
    abort();
  }
}
}
