/*
 * This file is part of lslidar_m10 driver.
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

#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <lslidar_m10_driver/lslidar_m10_driver.h>

namespace lslidar_m10_driver {

LslidarM10Driver::LslidarM10Driver(
        ros::NodeHandle& n, ros::NodeHandle& pn):
    nh(n),
    pnh(pn),
	socket_id_difop(-1),
    socket_id(-1){
    return;
}

LslidarM10Driver::~LslidarM10Driver() {
    (void) close(socket_id);
	(void) close(socket_id_difop);
    return;
}

bool LslidarM10Driver::loadParameters() {
    pnh.param<int>("versions", versions, 2);
	pnh.param("frame_id", frame_id, std::string("lslidar"));
	pnh.param("device_ip", device_ip_string, std::string("192.168.1.200"));
	pnh.param<int>("device_port", UDP_PORT_NUMBER, 2368);
	pnh.param("difop_ip", difop_ip_string, std::string("192.168.1.102"));
	pnh.param<int>("difop_port", UDP_PORT_NUMBER_DIFOP, 2369);

	inet_aton(device_ip_string.c_str(), &device_ip);
	ROS_INFO_STREAM("Opening UDP socket: address " << device_ip_string);
	ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
	ROS_INFO_STREAM("Opening UDP socket: difop address " << difop_ip_string);
	ROS_INFO_STREAM("Opening UDP socket: difop port " << UDP_PORT_NUMBER_DIFOP);
    if(versions == 1)           PACKET_SIZE = 92;
    else if(versions == 2)      PACKET_SIZE = 160;
    else if(versions == 3)      PACKET_SIZE = 162;
	return true;
}

bool LslidarM10Driver::createRosIO() {

  // ROS diagnostics
  diagnostics.setHardwareID("Lslidar_M10");
  const double diag_freq = 12*24;
  diag_max_freq = diag_freq;
  diag_min_freq = diag_freq;
  ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
	  diag_topic.reset(new TopicDiagnostic(
						 "lslidar_packets", diagnostics,
						 FrequencyStatusParam(&diag_min_freq, &diag_max_freq, 0.1, 10),
						 TimeStampStatusParam()));

    // Output
    packet_pub = nh.advertise<lslidar_m10_msgs::LslidarM10Packet>("lslidar_packet", 200);
    	//difop_sub_ = nh.subscribe("lslidar_packet_difop", 10, &LslidarM10Decoder::processDifop, (LslidarM10Decoder*)this);
	difop_switch = nh.subscribe<std_msgs::Int8>("lslidar_difop_switch", 1 ,&LslidarM10Driver::UDP_order,this);
	//std::string output_difop_topic;
	//pnh.param("output_difop_topic", output_difop_topic, std::string("lslidar_packet_difop"));
	difop_output_ = nh.advertise<lslidar_m10_msgs::LslidarM10Packet>("lslidar_packet_difop", 200);
	//difop_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&LslidarM10Driver::difopPoll, this)));

	return true;
}

bool LslidarM10Driver::openUDPPort() {
	socket_id_difop = socket(PF_INET, SOCK_DGRAM, 0);
    socket_id = socket(PF_INET, SOCK_DGRAM, 0);
    if (socket_id == -1) {
        perror("socket");
        return false;
    }
	if (socket_id_difop == -1) {
        perror("socket_difop");
        return false;
    }

    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(UDP_PORT_NUMBER);      // short, in network byte order
    ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

    if (bind(socket_id, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
        perror("bind");                 // TODO: ROS_ERROR errno
        return false;
    }
	my_addr.sin_port = htons(UDP_PORT_NUMBER_DIFOP);
	if (bind(socket_id_difop, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
        perror("bind");                 // TODO: ROS_ERROR errno
        return false;
    }
		
    if (fcntl(socket_id, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
        perror("non-block");
        return false;
    }
	 if (fcntl(socket_id_difop, F_SETFL, O_NONBLOCK|FASYNC) < 0) {
        perror("non-block-difop");
        return false;
    }
    is_start = false;
    return true;
}

bool LslidarM10Driver::initialize() {
    if (!loadParameters()) {
        ROS_ERROR("Cannot load all required ROS parameters...");
        return false;
    }

    if (!createRosIO()) {
        ROS_ERROR("Cannot create all ROS IO...");
        return false;
    }

    if (!openUDPPort()) {
        ROS_ERROR("Cannot open UDP port...");
        return false;
    }
    UDP_open(1);
    ROS_INFO("Initialised lslidar m10 without error");
    return true;
}

int LslidarM10Driver::getDifopPacket(lslidar_m10_msgs::LslidarM10PacketPtr& packet) {
    double time1 = ros::Time::now().toSec();
    struct pollfd fds[1];
    fds[0].fd = socket_id_difop;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 3000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    
    while (is_start)
    {

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
                //ROS_WARN("lslidar_difop poll() timeout");
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
        ssize_t nbytes = recvfrom(socket_id_difop, &packet->data[0], PACKET_SIZE,  0,
                (sockaddr*) &sender_address, &sender_address_len);
        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                perror("recvfail");
                ROS_INFO("recvfail");
                return 1;
            }
        }
        else if ((size_t) nbytes == PACKET_SIZE)
        {
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if( device_ip_string != "" && sender_address.sin_addr.s_addr != device_ip.s_addr )
			{
                continue;
			}
            else
                break; //done
        }
    }
    //printf("LslidarM10Driver::getDifopPacket\n");
    return 0;
}

void LslidarM10Driver::UDP_order(const std_msgs::Int8 msg) {
    int i = msg.data;
    sockaddr_in server_sai;
    server_sai.sin_family=AF_INET; //IPV4 协议族
    server_sai.sin_port=htons(UDP_PORT_NUMBER_DIFOP); 
    server_sai.sin_addr.s_addr=inet_addr(device_ip_string.c_str());
    int rtn = 0;
    for(int k = 0 ; k <10 ; k++)
    {   
        if(versions == 1){
            char data[188]= {0x00};
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[184] = 0x01;
            if(i == 1)      data[185] = 0x01;
            else            data[185] = 0x00;
            data[186] = 0xFA;
            data[187] = 0xFB;
            rtn = sendto(socket_id,data,188,0,(struct sockaddr *)&server_sai, sizeof(struct sockaddr));
        }
        else if(versions == 2){
            char data[188]= {0x00};
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[2] = 0x01;
            data[184] = 0x01;
            if(i == 1)      data[185] = 0x01;
            else            data[185] = 0x00;
            data[186] = 0xFA;
            data[187] = 0xFB;
            rtn = sendto(socket_id,data,188,0,(struct sockaddr *)&server_sai, sizeof(struct sockaddr));
        }
        else if (versions == 3){
            char data[11];
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[2] = 0x00;
            data[3] = 0x0B;
            data[4] = 0x55;
            data[5] = 0x12;
            data[6] = 0x02;
            if(i == 1)      
            {
                data[7] = 0x01;
                data[8] = 0x74;
            }
            else               
            {
                data[7] = 0x00;
                data[8] = 0x73;
            }
            data[9] = 0xFA;
            data[10] = 0xFB;
            rtn = sendto(socket_id,data,11,0,(struct sockaddr *)&server_sai, sizeof(struct sockaddr));
        }
        if (rtn < 0)
        {
            printf("start scan error !\n");
        }
        else 
        {
            if(i == 1)  usleep(3000000);
            if(i == 1)  is_start = true;
            else        is_start = false;
            return ;
        }   
    } 
    
      
    return ;
}

void LslidarM10Driver::UDP_open(int i) {
    printf("start scan  !\n");
    sockaddr_in server_sai;
    server_sai.sin_family=AF_INET; //IPV4 协议族
    server_sai.sin_port=htons(UDP_PORT_NUMBER_DIFOP); 
    server_sai.sin_addr.s_addr=inet_addr(device_ip_string.c_str());
    int rtn = 0;
    for(int k = 0 ; k <10 ; k++)
    {   
        if(versions == 1){
            char data[188]= {0x00};
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[184] = 0x01;
            if(i == 1)      data[185] = 0x01;
            else            data[185] = 0x00;
            data[186] = 0xFA;
            data[187] = 0xFB;
            rtn = sendto(socket_id,data,188,0,(struct sockaddr *)&server_sai, sizeof(struct sockaddr));
        }
        else if(versions == 2){
            char data[188]= {0x00};
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[2] = 0x01;
            data[184] = 0x01;
            if(i == 1)      data[185] = 0x01;
            else            data[185] = 0x00;
            data[186] = 0xFA;
            data[187] = 0xFB;
            rtn = sendto(socket_id,data,188,0,(struct sockaddr *)&server_sai, sizeof(struct sockaddr));
        }
        else if (versions == 3){
            char data[11];
            data[0] = 0xA5;
            data[1] = 0x5A;
            data[2] = 0x00;
            data[3] = 0x0B;
            data[4] = 0x55;
            data[5] = 0x12;
            data[6] = 0x02;
            if(i == 1)      
            {
                data[7] = 0x01;
                data[8] = 0x74;
            }
            else               
            {
                data[7] = 0x00;
                data[8] = 0x73;
            }
            data[9] = 0xFA;
            data[10] = 0xFB;
            rtn = sendto(socket_id,data,11,0,(struct sockaddr *)&server_sai, sizeof(struct sockaddr));
        }
        if (rtn < 0)
        {
            printf("start scan error !\n");
        }
        else 
        {
            if(i == 1)  usleep(3000000);
            if(i == 1)  is_start = true;
            else        is_start = false;
            return ;
        }   
    } 
      
    return ;
}

int LslidarM10Driver::getPacket( lslidar_m10_msgs::LslidarM10PacketPtr& packet) {
    double time1 = ros::Time::now().toSec();
    struct pollfd fds[1];
    fds[0].fd = socket_id;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 2000; // one second (in msec)

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);
    while (is_start)
    {
        // Unfortunately, the Linux kernel recvfrom() implementation
        // uses a non-interruptible sleep() when waiting for data,
        // which would cause this method to hang if the device is not
        // providing data.  We poll() the device first to make sure
        // the recvfrom() will not block.
        //
        // Note, however, that there is a known Linux kernel bug:
        //
        //   Under Linux, select() may report a socket file descriptor
        //   as "ready for reading", while nevertheless a subsequent
        //   read blocks.  This could for example happen when data has
        //   arrived but upon examination has wrong checksum and is
        //   discarded.  There may be other circumstances in which a
        //   file descriptor is spuriously reported as ready.  Thus it
        //   may be safer to use O_NONBLOCK on sockets that should not
        //   block.
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
        ssize_t nbytes = recvfrom(socket_id, &packet->data[0], PACKET_SIZE,  0,
                (sockaddr*) &sender_address, &sender_address_len);

        if (nbytes < 0)
        {
            if (errno != EWOULDBLOCK)
            {
                perror("recvfail");
                ROS_INFO("recvfail");
                return 1;
            }
        }
        else if ((size_t) nbytes == PACKET_SIZE)
        {
            // read successful,
            // if packet is not from the lidar scanner we selected by IP,
            // continue otherwise we are done
            if( device_ip_string != "" && sender_address.sin_addr.s_addr != device_ip.s_addr )
                continue;
            else
                break; //done
        }
    }

    // Average the times at which we begin and end reading.  Use that to
    // estimate when the scan occurred.
    double time2 = ros::Time::now().toSec();
    packet->stamp = ros::Time((time2 + time1) / 2.0);
    //printf("LslidarM10Driver::getPacket\n");
    return 0;
}

bool LslidarM10Driver::polling()
{
    // Allocate a new shared pointer for zero-copy sharing with other nodelets.
    lslidar_m10_msgs::LslidarM10PacketPtr packet(
                new lslidar_m10_msgs::LslidarM10Packet());
				
	struct timeval tv;
	int last_usec,now_usec;

	while (true)
    {
        // keep reading until full packet received
        int rc = getPacket(packet);
        if (rc == 0) break;       // got a full packet?
        if (rc < 0) return false; // end of file reached?
    }
	
	packet_pub.publish(*packet);

    // notify diagnostics that a message has been published, updating
    // its status
    diag_topic->tick(packet->stamp);
    diagnostics.update();
    //printf("LslidarM10Driver::polling\n");
    return true;
}

void LslidarM10Driver::difopPoll()
{
    // reading and publishing scans as fast as possible.
	lslidar_m10_msgs::LslidarM10PacketPtr packets(
                new lslidar_m10_msgs::LslidarM10Packet());
				
    while (ros::ok())
    {
        // keep reading
		 int rc = getDifopPacket(packets);
        if (rc == 0)
        {
				difop_output_.publish(*packets);
        }
        if (rc < 0)
            return;  // end of file reached?
        ros::spinOnce();
    }
}

} // namespace lslidar_driver
