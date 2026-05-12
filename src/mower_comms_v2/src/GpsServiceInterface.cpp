//
// Created by clemens on 30.11.24.
//

#include "GpsServiceInterface.h"

#include <nmea_msgs/Sentence.h>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "GeographicLib/DMS.hpp"
#include "robot_localization/navsat_conversions.h"

GpsServiceInterface::GpsServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                                         const ros::Publisher& absolute_pose_publisher,
                                         const ros::Publisher& nmea_publisher,
                                         const ros::Publisher& gps_status_publisher, double datum_lat,
                                         double datum_long, double datum_height, uint32_t baud_rate,
                                         const std::string& protocol, uint8_t port_index, bool absolute_coords)
    : GpsServiceInterfaceBase(service_id, ctx),
      absolute_pose_publisher_(absolute_pose_publisher),
      nmea_publisher_(nmea_publisher),
      gps_status_publisher_(gps_status_publisher),
      baud_rate_(baud_rate),
      protocol_(protocol),
      port_index_(port_index),
      absolute_coords_(absolute_coords) {
  RobotLocalization::NavsatConversions::LLtoUTM(datum_lat, datum_long, datum_n_, datum_e_, datum_zone_);
  datum_u_ = datum_height;
}

void GpsServiceInterface::SendNMEA(double lat_in, double lon_in) {
  using namespace GeographicLib;
  // only send every 10 seconds, this will be more than needed
  static ros::Time last_vrs_feedback(0.0);
  static nmea_msgs::Sentence vrs_msg{};
  if ((ros::Time::now() - last_vrs_feedback).toSec() < 10.0) {
    return;
  }
  last_vrs_feedback = ros::Time::now();

  auto lat = GeographicLib::DMS::Encode(lat_in, GeographicLib::DMS::component::MINUTE, 4,
                                        GeographicLib::DMS::flag::LATITUDE, ';');
  auto lon = GeographicLib::DMS::Encode(lon_in, GeographicLib::DMS::component::MINUTE, 4,
                                        GeographicLib::DMS::flag::LONGITUDE, ';');

  // remove separator char
  boost::erase_all(lat, ";");
  boost::erase_all(lon, ";");

  auto lat_hemisphere = lat.substr(lat.length() - 1, 1);
  auto lon_hemisphere = lon.substr(lon.length() - 1, 1);

  std::stringstream message_ss;
  auto time_facet = new boost::posix_time::time_facet("%H%M%s");

  message_ss.imbue(std::locale(message_ss.getloc(), time_facet));
  message_ss << "GPGGA," << ros::Time::now().toBoost() << "," << lat.substr(0, lat.length() - 1) << ","
             << lat_hemisphere << "," << lon.substr(0, lon.length() - 1) << "," << lon_hemisphere
             << ",1,0,0,0,M,0,M,0000,";

  std::string message_content = message_ss.str();

  uint8_t checksum = 0;
  for (const auto c : message_content) {
    checksum ^= c;
  }

  std::stringstream final_message_ss;
  final_message_ss << "$" << message_content << "*" << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                   << (int)checksum;

  vrs_msg.header.frame_id = "gps";
  vrs_msg.header.seq++;
  vrs_msg.header.stamp = ros::Time::now();
  vrs_msg.sentence = final_message_ss.str();
  nmea_publisher_.publish(vrs_msg);
}

bool GpsServiceInterface::OnConfigurationRequested(uint16_t service_id) {
  StartTransaction(true);
  SetRegisterBaudrate(baud_rate_);
  if (protocol_ == "UBX") {
    SetRegisterProtocol(ProtocolType::UBX);
  } else if (protocol_ == "NMEA") {
    SetRegisterProtocol(ProtocolType::NMEA);
  } else {
    ROS_ERROR_STREAM("Invalid Protocol: " << protocol_);
  }
  if (port_index_ > 0) {
    SetRegisterUart(port_index_);
  }
  CommitTransaction();
  return true;
}

void GpsServiceInterface::OnTransactionStart(uint64_t timestamp) {
  pose_msg_.header.frame_id = "gps";
  pose_msg_.header.stamp = ros::Time::now();
  pose_msg_.header.seq++;
  pose_msg_.motion_vector_valid = false;
  pose_msg_.orientation_valid = false;
  pose_msg_.sensor_stamp = timestamp / 1000;
  pose_msg_.flags = 0;
}

void GpsServiceInterface::OnPositionChanged(const double* new_value, uint32_t length) {
  if (length != 3) {
    ROS_INFO_STREAM("OnPositionChanged called with length " << length);
    return;
  }
  last_position_update_ = ros::Time::now();
  if (absolute_coords_) {
    SendNMEA(new_value[0], new_value[1]);
    double e, n;
    std::string zone;
    RobotLocalization::NavsatConversions::LLtoUTM(new_value[0], new_value[1], n, e, zone);
    pose_msg_.pose.pose.position.x = e - datum_e_;
    pose_msg_.pose.pose.position.y = n - datum_n_;
    pose_msg_.pose.pose.position.z = new_value[2] - datum_u_;
  } else {
    double n = new_value[1] + datum_n_;
    double e = new_value[0] + datum_e_;
    double lat, lng;
    RobotLocalization::NavsatConversions::UTMtoLL(n, e, datum_zone_, lat, lng);
    SendNMEA(lat, lng);
    pose_msg_.pose.pose.position.x = new_value[0];
    pose_msg_.pose.pose.position.y = new_value[1];
    pose_msg_.pose.pose.position.z = new_value[2];
  }
}

void GpsServiceInterface::OnPositionHorizontalAccuracyChanged(const double& new_value) {
  pose_msg_.position_accuracy = static_cast<float>(new_value);
}

void GpsServiceInterface::OnFixTypeChanged(const char* new_value, uint32_t length) {
  pose_msg_.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_RTK;
  std::string type(new_value, length);
  if (type == "FIX") {
    pose_msg_.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED;
    last_fix_type_ = 5;  // RTK fixed
  } else if (type == "FLOAT") {
    pose_msg_.flags |= xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FLOAT;
    last_fix_type_ = 4;  // RTK float
  } else if (type == "NONE" || type == "INVALID" || length == 0) {
    // Firmware explicitly reports loss of fix — propagate that immediately
    // instead of holding the previous "FIX"/"FLOAT" until the position
    // watchdog in OnTransactionEnd fires.
    last_fix_type_ = 0;
  } else {
    // Anything else the firmware reports — assume a 3D fix is at least there
    // (DGPS / 3D / 2D distinction isn't exposed by the v2 service). The
    // frontend mostly uses 5/4/anything-else for the RTK chip, so a single
    // "non-RTK fix" bucket is enough.
    last_fix_type_ = 2;
  }
}

void GpsServiceInterface::OnMotionVectorENUChanged(const double* new_value, uint32_t length) {
  if (length != 3) {
    ROS_INFO_STREAM("OnMotionVectorENUChanged called with length " << length);
    return;
  }
  pose_msg_.motion_vector_valid = true;
  pose_msg_.motion_vector.x = new_value[0];
  pose_msg_.motion_vector.y = new_value[1];
  pose_msg_.motion_vector.z = new_value[2];
}

void GpsServiceInterface::OnMotionHeadingAndAccuracyChanged(const double* new_value, uint32_t length) {
  if (length != 2) {
    ROS_INFO_STREAM("OnMotionHeadingAndAccuracyChanged called with length " << length);
    return;
  }
  pose_msg_.motion_heading = new_value[0];
  pose_msg_.motion_vector_valid = true;
}

void GpsServiceInterface::OnVehicleHeadingAndAccuracyChanged(const double* new_value, uint32_t length) {
  if (length != 2) {
    ROS_INFO_STREAM("OnVehicleHeadingAndAccuracyChanged called with length " << length);
    return;
  }
  pose_msg_.vehicle_heading = new_value[0];
  pose_msg_.orientation_accuracy = new_value[1];
  pose_msg_.orientation_valid = true;
}

void GpsServiceInterface::OnTransactionEnd() {
  absolute_pose_publisher_.publish(pose_msg_);

  // Stale-fix watchdog: when the firmware stops delivering positions (antenna
  // unplugged, indoors, GpsService crash) we never get a "NONE" fix-type
  // event — the last known FIX/FLOAT just stops getting refreshed. Without
  // this guard the dashboard happily shows "RTK Fixed" for hours after the
  // receiver went blind. Three seconds is generous for the 5..10 Hz update
  // rate of an F9P/F9R while still hiding the obvious failure mode.
  static constexpr double kFixStaleSeconds = 3.0;
  if (last_position_update_.isZero() ||
      (ros::Time::now() - last_position_update_).toSec() > kFixStaleSeconds) {
    last_fix_type_ = 0;
  }

  xbot_msgs::GpsStatus status_msg;
  status_msg.header.stamp = pose_msg_.header.stamp;
  status_msg.header.frame_id = "gps";
  status_msg.fix_type = last_fix_type_;
  // numSV and pDOP are not exposed by the v2 firmware GpsService — the
  // frontend treats them as N/A when zero.
  status_msg.satellite_count = 0;
  status_msg.pdop = 0.0f;
  gps_status_publisher_.publish(status_msg);
}
