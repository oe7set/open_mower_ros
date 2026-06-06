// gnss_detail_parser_node
//
// Connects to the v2 firmware GPS debug TCP interface (raw NMEA/UBX mirror),
// parses the per-satellite / RF / DOP detail off-board, and publishes
// xbot_msgs/GnssDetail on ll/position/gnss_detail — the exact topic
// xbot_monitoring bridges to the app's gnss/stream. Keeping this parsing on the
// CM4 lets the firmware emit only navigation-critical outputs, which removed the
// per-sentence transaction flood that previously hung the GPS thread.

#include <ros/ros.h>
#include <xbot_msgs/GnssDetail.h>
#include <xbot_msgs/Satellite.h>

#include <cerrno>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

extern "C" {
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
}

#include "parser/nmea_parser.h"
#include "parser/ubx_parser.h"

namespace {

using gnss_detail_parser::GnssDetailState;
using gnss_detail_parser::Parser;

ros::Publisher g_detail_pub;

// Map our accumulated detail state to the ROS message and publish it.
void PublishDetail(const GnssDetailState& s) {
  xbot_msgs::GnssDetail msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "gps";

  msg.satellites.reserve(s.satellites.size());
  for (const auto& sat : s.satellites) {
    xbot_msgs::Satellite o;
    o.gnss_id = sat.gnss_id;
    o.sv_id = sat.sv_id;
    o.cn0 = sat.cn0;
    o.band = sat.band;
    o.elevation = sat.elevation;
    o.azimuth = sat.azimuth;
    o.flags = static_cast<uint8_t>((sat.used ? xbot_msgs::Satellite::FLAG_USED : 0) |
                                   (sat.healthy ? xbot_msgs::Satellite::FLAG_HEALTHY : 0));
    msg.satellites.push_back(o);
  }
  msg.sats_visible = s.sats_visible;
  msg.sats_used = s.sats_used;

  msg.gdop = s.gdop;
  msg.pdop = s.pdop;
  msg.hdop = s.hdop;
  msg.vdop = s.vdop;
  msg.tdop = s.tdop;

  msg.fix_type = s.fix_type;
  msg.rtk_type = s.rtk_type;

  msg.lat = s.lat;
  msg.lon = s.lon;
  msg.height = s.height;
  msg.h_acc = s.h_acc;
  msg.v_acc = s.v_acc;

  msg.vel_e = s.vel_e;
  msg.vel_n = s.vel_n;
  msg.vel_u = s.vel_u;

  msg.vehicle_heading = s.vehicle_heading;
  msg.motion_heading = s.motion_heading;

  msg.correction_age = s.correction_age;
  msg.baseline_len = s.baseline_len;
  msg.heading_accuracy = s.heading_accuracy;
  msg.solution_status = s.solution_status;
  msg.elevation_cutoff = s.elevation_cutoff;
  msg.antenna_agc = s.antenna_agc;
  msg.jamming = s.jamming;

  g_detail_pub.publish(msg);
}

// Resolve + connect a TCP client socket to host:port. Returns fd or -1.
int TcpConnect(const std::string& host, const std::string& port) {
  struct addrinfo hints {};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  struct addrinfo* addrs = nullptr;
  if (getaddrinfo(host.c_str(), port.c_str(), &hints, &addrs) != 0 || addrs == nullptr) {
    return -1;
  }
  int fd = socket(addrs->ai_family, addrs->ai_socktype, addrs->ai_protocol);
  if (fd < 0) {
    freeaddrinfo(addrs);
    return -1;
  }
  struct timeval timeout {};
  timeout.tv_sec = 5;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  int flag = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
  if (connect(fd, addrs->ai_addr, addrs->ai_addrlen) != 0) {
    close(fd);
    freeaddrinfo(addrs);
    return -1;
  }
  freeaddrinfo(addrs);
  return fd;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "gnss_detail_parser");
  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  std::string host;
  // Default matches the xCore debug-port address used by `openmower expose-gps`.
  // The launch file overrides this via the firmware_ip param / OM_GNSS_FIRMWARE_IP.
  private_nh.param<std::string>("firmware_ip", host, "172.16.78.150");
  int port_i = 0;
  private_nh.param("firmware_port", port_i, 10000);
  std::string protocol;
  private_nh.param<std::string>("protocol", protocol, "UBX");
  bool send_config = true;
  private_nh.param("send_config", send_config, true);

  g_detail_pub = n.advertise<xbot_msgs::GnssDetail>("ll/position/gnss_detail", 5);

  ROS_INFO_STREAM("gnss_detail_parser: connecting to " << host << ":" << port_i << " protocol=" << protocol);

  while (ros::ok()) {
    std::unique_ptr<Parser> parser;
    if (protocol == "NMEA") {
      parser = std::make_unique<gnss_detail_parser::NmeaParser>();
    } else {
      parser = std::make_unique<gnss_detail_parser::UbxParser>();
    }
    parser->SetEpochCallback(&PublishDetail);

    int fd = TcpConnect(host, std::to_string(port_i));
    if (fd < 0) {
      ROS_WARN_THROTTLE(10.0, "gnss_detail_parser: connect failed, retrying...");
      ros::Duration(2.0).sleep();
      continue;
    }
    ROS_INFO("gnss_detail_parser: connected, raw stream open");
    parser->Reset();

    // Ask the receiver to emit the detail messages we parse (the firmware no
    // longer requests them). Best-effort; ignore write errors.
    if (send_config) {
      for (const auto& cmd : parser->StartupCommands()) {
        ssize_t w = write(fd, cmd.data(), cmd.size());
        (void)w;
      }
    }

    // Force a reconnect if the link stays silent too long. The TCP connection
    // can stay half-open (firmware GPS thread wedged, or the single-client debug
    // port held by another consumer) while read() just times out forever; a
    // fresh connect re-arms the receiver detail-log config and frees a stuck
    // port slot. SO_RCVTIMEO is 5 s, so 6 idle reads ≈ 30 s of silence.
    constexpr int kMaxIdleReads = 6;
    int idle_reads = 0;
    std::vector<uint8_t> buf(2048);
    while (ros::ok()) {
      ssize_t got = read(fd, buf.data(), buf.size());
      if (got > 0) {
        idle_reads = 0;
        parser->ProcessBytes(buf.data(), static_cast<size_t>(got));
      } else if (got == 0) {
        ROS_WARN("gnss_detail_parser: stream closed by firmware, reconnecting");
        break;
      } else {
        // Timeout (EAGAIN/EWOULDBLOCK) is normal; any other error -> reconnect.
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          ROS_WARN_STREAM("gnss_detail_parser: read error (" << strerror(errno) << "), reconnecting");
          break;
        }
        if (++idle_reads >= kMaxIdleReads) {
          ROS_WARN("gnss_detail_parser: no data for ~30s, reconnecting");
          break;
        }
      }
      ros::spinOnce();
    }

    close(fd);
    if (ros::ok()) ros::Duration(1.0).sleep();
  }

  return 0;
}
