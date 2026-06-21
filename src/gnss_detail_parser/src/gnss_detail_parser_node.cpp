// gnss_detail_parser_node
//
// Connects to the v2 firmware GPS debug TCP interface (raw NMEA/UBX mirror),
// parses the per-satellite / RF / DOP detail off-board, and publishes
// xbot_msgs/GnssDetail on ll/position/gnss_detail — the exact topic
// xbot_monitoring bridges to the app's gnss/stream. Keeping this parsing on the
// CM4 lets the firmware emit only navigation-critical outputs, which removed the
// per-sentence transaction flood that previously hung the GPS thread.

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <xbot_msgs/GnssDetail.h>
#include <xbot_msgs/Satellite.h>

#include <algorithm>
#include <atomic>
#include <cctype>
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
// Wall time of the last published epoch; zero = none yet. Used by the read loop
// to warn when bytes flow but no epoch is framed (the classic protocol mismatch).
ros::Time g_last_epoch_time(0);

// Enabled = this parser owns the firmware's single-client debug port (10000).
// `openmower expose-gps` calls ~set_enabled false to hand that port to u-center,
// then ~set_enabled true to give it back. When false the read loop closes its
// socket and stops reconnecting, releasing the port. Atomic because a future
// AsyncSpinner could run the callback off the loop thread.
std::atomic<bool> g_enabled{true};
// Wall time of the last set_enabled request; drives the safety watchdog that
// auto-resumes if a crashed expose-gps never re-enables us.
ros::Time g_last_enable_change(0);

// ~set_enabled service: pause (data=false) releases the single-client debug port
// so `expose-gps` / u-center can use it; resume (data=true) reconnects.
bool OnSetEnabled(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  g_enabled.store(req.data);
  g_last_enable_change = ros::Time::now();
  res.success = true;
  res.message =
      req.data ? "gnss_detail_parser enabled (will reconnect)" : "gnss_detail_parser paused (port 10000 released)";
  ROS_INFO_STREAM("gnss_detail_parser: set_enabled -> " << (req.data ? "true" : "false"));
  return true;
}

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
  g_last_epoch_time = ros::Time::now();
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
  // Protocol resolution: the parser MUST match the protocol the firmware
  // (mower_comms_v2) configured the receiver for, otherwise we run the wrong
  // parser against the raw stream, never frame an epoch and publish nothing.
  // Order: 1) explicit private ~protocol override (launch arg / OM_GNSS_PROTOCOL),
  //        2) the single source of truth /ll/services/gps/protocol (YAML),
  //        3) UBX fallback (matches openmower_defaults_v2.yaml).
  std::string protocol;
  private_nh.param<std::string>("protocol", protocol, "");
  if (protocol.empty()) {
    n.param<std::string>("/ll/services/gps/protocol", protocol, "UBX");
    ROS_INFO_STREAM("gnss_detail_parser: protocol from /ll/services/gps/protocol = " << protocol);
  } else {
    ROS_INFO_STREAM("gnss_detail_parser: protocol overridden via ~protocol = " << protocol);
  }
  // Normalize case so "nmea"/"Nmea" still selects the NMEA parser (YAML uses uppercase).
  std::transform(protocol.begin(), protocol.end(), protocol.begin(),
                 [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  bool send_config = true;
  private_nh.param("send_config", send_config, true);

  // Safety watchdog: if paused for longer than this many seconds without a fresh
  // set_enabled request, auto-resume so a crashed/killed `expose-gps` cannot leave
  // the app GNSS page dead forever. 0 disables the watchdog.
  double resume_timeout = 0.0;
  private_nh.param("pause_watchdog_timeout", resume_timeout, 300.0);

  g_detail_pub = n.advertise<xbot_msgs::GnssDetail>("ll/position/gnss_detail", 5);
  ros::ServiceServer set_enabled_srv = private_nh.advertiseService("set_enabled", OnSetEnabled);
  g_last_enable_change = ros::Time::now();

  ROS_INFO_STREAM("gnss_detail_parser: connecting to " << host << ":" << port_i << " protocol=" << protocol);

  while (ros::ok()) {
    // Paused: hold no socket (releasing port 10000) and do not reconnect. Keep
    // servicing callbacks so we can be re-enabled. The watchdog auto-resumes if
    // the pause is never refreshed (crashed expose-gps).
    if (!g_enabled.load()) {
      if (resume_timeout > 0.0 && !g_last_enable_change.isZero() &&
          (ros::Time::now() - g_last_enable_change).toSec() > resume_timeout) {
        ROS_WARN("gnss_detail_parser: pause watchdog expired (%.0fs), auto-resuming", resume_timeout);
        g_enabled.store(true);
        g_last_enable_change = ros::Time::now();
      } else {
        ros::spinOnce();
        ros::Duration(0.2).sleep();
        continue;
      }
    }

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
    // g_enabled in the guard: when expose-gps pauses us, we drop out of the read
    // loop and close(fd) below, releasing port 10000. Worst-case handoff latency
    // is one SO_RCVTIMEO (5 s) — left as-is so the kMaxIdleReads (~30 s) heuristic
    // above stays valid.
    while (ros::ok() && g_enabled.load()) {
      ssize_t got = read(fd, buf.data(), buf.size());
      if (got > 0) {
        idle_reads = 0;
        parser->ProcessBytes(buf.data(), static_cast<size_t>(got));
        // Data is flowing but we are not framing epochs — the usual cause is a
        // protocol mismatch (wrong parser for this receiver's output). Surface it
        // instead of failing silently.
        constexpr double kEpochWarnSec = 15.0;
        if (g_last_epoch_time.isZero()) {
          ROS_WARN_THROTTLE(kEpochWarnSec,
                            "gnss_detail_parser: receiving bytes but no GnssDetail epoch yet "
                            "(protocol=%s) — check that the receiver protocol matches "
                            "/ll/services/gps/protocol",
                            protocol.c_str());
        } else if ((ros::Time::now() - g_last_epoch_time).toSec() > kEpochWarnSec) {
          ROS_WARN_THROTTLE(kEpochWarnSec,
                            "gnss_detail_parser: no GnssDetail epoch for >%.0fs despite live stream "
                            "(protocol=%s)",
                            kEpochWarnSec, protocol.c_str());
        }
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

    close(fd);  // releases port 10000
    // Only back off before reconnecting; if we were paused, the outer loop's
    // paused branch handles the wait without this extra delay.
    if (ros::ok() && g_enabled.load()) ros::Duration(1.0).sleep();
  }

  return 0;
}
