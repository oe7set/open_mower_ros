// Created by Clemens Elflein on 3/28/22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//

#include <dynamic_reconfigure/client.h>
#include <mower_msgs/ESCStatus.h>
#include <mower_msgs/Power.h>
#include <xbot_msgs/SensorDataString.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "event_publisher/event_publisher.hpp"
#include "mower_logic/MowerLogicConfig.h"
#include "mower_logic/PowerConfig.h"
#include "mower_msgs/HighLevelStatus.h"
#include "mower_msgs/Status.h"
#include "ros/ros.h"
#include "xbot_msgs/AbsolutePose.h"
#include "xbot_msgs/GpsStatus.h"
#include "xbot_msgs/RobotState.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/SensorInfo.h"

ros::Publisher state_pub;
xbot_msgs::RobotState state;

ros::NodeHandle* n;

ros::NodeHandle* paramNh;

dynamic_reconfigure::Client<mower_logic::MowerLogicConfig>* logicReconfigClient;
dynamic_reconfigure::Client<ll::PowerConfig>* powerReconfigClient;
mower_logic::MowerLogicConfig mower_logic_config;
ll::PowerConfig power_config;

typedef const mower_msgs::Status::ConstPtr StatusPtr;

// Sensor configuration
struct SensorConfig {
  std::string name;     // Speaking name, used in sensor widget
  std::string unit;     // Unit like A, V, ...
  uint8_t value_desc;   // Voltage, Current, RPM, ...
  uint8_t sensor_type;  // Double, String, ...
  std::function<double(StatusPtr)> getStatusSensorValueCB = nullptr;
  std::function<void(SensorConfig& sensor_config)> setSensorLimitsCB = nullptr;
  std::string param_path = "";              // Path to parameters
  std::function<bool()> existCB = nullptr;  // nullptr = no callback for exist check = enabled
  xbot_msgs::SensorInfo si;                 // SensorInfo Msg
  ros::Publisher si_pub;                    // SensorInfo publisher
  ros::Publisher data_pub;                  // Sensor-data publisher
};

// Forward declare set_limits_* callback functions
void set_limits_battery_v(SensorConfig& sensor_config);

void set_limits_charge_current(SensorConfig& sensor_config);

void set_limits_charge_v(SensorConfig& sensor_config);

void set_limits_esc_temp(SensorConfig& sensor_config);

void set_limits_mow_motor_current(SensorConfig& sensor_config);

void set_limits_mow_motor_rpm(SensorConfig& sensor_config);

void set_limits_mow_motor_temp(SensorConfig& sensor_config);

// Place all sensors in a key=sensor.id -> SensorConfig map
// clang-format off
std::map<std::string, SensorConfig> sensor_configs{
  {"om_v_charge", {"V Charge", "V", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE, xbot_msgs::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_charge_v}},
  {"om_v_battery", {"V Battery", "V", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE, xbot_msgs::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_battery_v}},
  {"om_charge_current", {"Charge Current", "A", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT, xbot_msgs::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_charge_current, "", [](){ return !paramNh->param("/mower_logic/ignore_charging_current", false); }}},
  {"om_charge_state", {"Charge State", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_STRING, nullptr}},
  {"om_left_esc_temp", {"Left ESC Temp", "deg.C", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_esc_temp, "left_xesc"}},
  {"om_right_esc_temp", {"Right ESC Temp", "deg.C", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::SensorInfo::TYPE_DOUBLE, nullptr, &set_limits_esc_temp, "right_xesc"}},
  {"om_mow_esc_temp", {"Mow ESC Temp", "deg.C", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::SensorInfo::TYPE_DOUBLE, [](StatusPtr msg) { return msg->mower_esc_temperature; }, &set_limits_esc_temp, "mower_xesc"}},
  {"om_mow_motor_temp", {"Mow Motor Temp", "deg.C", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE, xbot_msgs::SensorInfo::TYPE_DOUBLE, [](StatusPtr msg) { return msg->mower_motor_temperature; }, &set_limits_mow_motor_temp, "mower_xesc", [](){ return paramNh->param("mower_xesc/has_motor_temp", true); }}},
  {"om_mow_motor_current", {"Mow Motor Current", "A", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT, xbot_msgs::SensorInfo::TYPE_DOUBLE, [](StatusPtr msg) { return msg->mower_esc_current; }, &set_limits_mow_motor_current, "mower_xesc"}},
  {"om_mow_motor_rpm", {"Mow Motor Revolutions", "rpm", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_RPM, xbot_msgs::SensorInfo::TYPE_DOUBLE, [](StatusPtr msg) { return msg->mower_motor_rpm; }, &set_limits_mow_motor_rpm, "mower_xesc"}},
  {"om_gps_accuracy", {"GPS Accuracy", "m", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_DISTANCE, xbot_msgs::SensorInfo::TYPE_DOUBLE}},
  {"om_gps_quality", {"GPS Quality", "%", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT, xbot_msgs::SensorInfo::TYPE_DOUBLE}},
  {"om_gps_satellites", {"GPS Satellites", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_DOUBLE}},
  {"om_gps_pdop", {"GPS PDOP", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_DOUBLE}},
  {"om_gps_fix_type", {"GPS Fix Type", "", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_DOUBLE}},
  {"om_gps_heading_accuracy", {"GPS Heading Accuracy", "deg", xbot_msgs::SensorInfo::VALUE_DESCRIPTION_UNKNOWN, xbot_msgs::SensorInfo::TYPE_DOUBLE}},
};
// clang-format on

void status_received(StatusPtr& msg) {
  // Rate limit to 2Hz
  static ros::Time last_update{0};
  if ((msg->stamp - last_update).toSec() < 0.5) return;
  last_update = msg->stamp;

  xbot_msgs::SensorDataDouble sensor_data;
  sensor_data.stamp = msg->stamp;

  for (auto& sc_pair : sensor_configs) {
    // Skip if sensor doesn't exists or is disabled
    if (sc_pair.second.existCB && !sc_pair.second.existCB()) continue;

    if (sc_pair.second.getStatusSensorValueCB) {
      sensor_data.data = sc_pair.second.getStatusSensorValueCB(msg);
      sc_pair.second.data_pub.publish(sensor_data);
    }
  }
  state.rain_detected = msg->rain_detected;
}

void high_level_status(const mower_msgs::HighLevelStatus::ConstPtr& msg) {
  state.gps_percentage = msg->gps_quality_percent;
  state.current_state = msg->state_name;
  state.current_sub_state = msg->sub_state_name;
  state.current_area = msg->current_area;
  state.current_path = msg->current_path;
  state.current_path_index = msg->current_path_index;
  state.battery_percentage = msg->battery_percent;
  state.emergency = msg->emergency;
  state.is_charging = msg->is_charging;

  // Mirror GPS quality into the sensor pipeline so the Sensors page can chart
  // the long-term percentage. HighLevelStatus does not carry its own stamp,
  // so use ros::Time::now() — same convention as gps_status_received.
  {
    auto sc_it = sensor_configs.find("om_gps_quality");
    if (sc_it != std::end(sensor_configs)) {
      xbot_msgs::SensorDataDouble sensor_data;
      sensor_data.stamp = ros::Time::now();
      sensor_data.data = static_cast<double>(msg->gps_quality_percent);
      sc_it->second.data_pub.publish(sensor_data);
    }
  }

  state_pub.publish(state);
}

// RTK fix-type tracking. We only emit lost/restored events on a sustained
// transition (3 seconds) to avoid spam from momentary fix drops in heavy
// canopy. State is the canonical "good RTK" boolean (fix_type >= 4) plus the
// timestamp of the last contradictory observation.
namespace {
const ros::Duration kRtkTransitionThreshold(3.0);
bool rtk_currently_good = true;  // assume good until proven otherwise
bool rtk_pending_state = true;   // candidate next state
ros::Time rtk_pending_since{0, 0};
}  // namespace

void gps_status_received(const xbot_msgs::GpsStatus::ConstPtr& msg) {
  state.gps_fix_type = msg->fix_type;
  state.gps_satellite_count = msg->satellite_count;
  state.gps_pdop = msg->pdop;

  // Forward the numeric GPS-quality fields into the sensor pipeline so they
  // can be charted on the Sensors page. xbot_msgs::GpsStatus has no stamp
  // field, so use ros::Time::now() — matches the existing rate of GpsStatus
  // (~1 Hz on real hardware) closely enough for charting.
  {
    const ros::Time stamp = ros::Time::now();
    auto publish = [&stamp](const std::string& sensor_id, double value) {
      auto sc_it = sensor_configs.find(sensor_id);
      if (sc_it == std::end(sensor_configs)) return;
      xbot_msgs::SensorDataDouble sensor_data;
      sensor_data.stamp = stamp;
      sensor_data.data = value;
      sc_it->second.data_pub.publish(sensor_data);
    };
    publish("om_gps_satellites", static_cast<double>(msg->satellite_count));
    publish("om_gps_pdop", static_cast<double>(msg->pdop));
    publish("om_gps_fix_type", static_cast<double>(msg->fix_type));
  }

  // GpsStatus.fix_type: 0=no fix, 1=2D, 2=3D, 3=DGPS/SBAS, 4=RTK float, 5=RTK fixed.
  // Treat fix_type >= 4 as "RTK quality"; anything below is the loss case.
  constexpr uint8_t kRtkFloatThreshold = 4;
  const bool good_now = msg->fix_type >= kRtkFloatThreshold;
  const ros::Time now = ros::Time::now();
  if (good_now == rtk_currently_good) {
    rtk_pending_since = ros::Time(0, 0);
    return;
  }
  if (good_now != rtk_pending_state) {
    rtk_pending_state = good_now;
    rtk_pending_since = now;
    return;
  }
  if (rtk_pending_since.isZero() || (now - rtk_pending_since) < kRtkTransitionThreshold) {
    return;
  }
  rtk_currently_good = good_now;
  rtk_pending_since = ros::Time(0, 0);
  if (good_now) {
    open_mower::events::EventPublisher::info(
        "gps.rtk_restored", "RTK fix restored",
        {{"fix_type", static_cast<int>(msg->fix_type)}, {"satellites", static_cast<int>(msg->satellite_count)}});
  } else {
    open_mower::events::EventPublisher::warning(
        "gps.rtk_lost", "RTK fix lost",
        {{"fix_type", static_cast<int>(msg->fix_type)}, {"satellites", static_cast<int>(msg->satellite_count)}});
  }
}

// WLAN-presence tracking. WLAN is "present" when /proc/net/wireless yields a
// non-zero dBm reading. Same debounce idea as RTK above — 5 s sustained
// transition before we surface lost/restored, so a single dropped read does
// not page the user.
namespace {
const ros::Duration kWifiTransitionThreshold(5.0);
bool wifi_currently_present = true;
bool wifi_pending_state = true;
ros::Time wifi_pending_since{0, 0};
bool wifi_seen_once = false;  // suppress lost-event before we ever had a reading
}  // namespace

// Reads /proc/net/wireless (no privileged calls or external binaries needed).
// On a host without WLAN (e.g. Ethernet-only setups) the file either misses
// or has only the header lines; we fall back to dbm=0, quality=0 so the
// frontend can render an "N/A" state instead of crashing on parse errors.
//
// /proc/net/wireless format (Linux kernel):
//   Inter-| sta-|   Quality        |   Discarded packets               | Missed | WE
//    face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon | 22
//      wlan0: 0000   54.  -56.  -256        0      0      0      0      0        0
// We take 'link' as the 0..70 quality and 'level' as signed dBm.
static void update_wifi_stats() {
  std::ifstream f("/proc/net/wireless");
  if (!f.is_open()) {
    state.wifi_signal_dbm = 0;
    state.wifi_link_quality = 0.0f;
    return;
  }
  std::string line;
  // Skip the two header lines; the first interface entry comes after them.
  std::getline(f, line);
  std::getline(f, line);
  if (!std::getline(f, line)) {
    state.wifi_signal_dbm = 0;
    state.wifi_link_quality = 0.0f;
    return;
  }
  // Drop the leading whitespace and the "iface:" prefix.
  std::istringstream iss(line);
  std::string iface;
  iss >> iface;  // "wlan0:"
  int status;
  double link, level;
  if (!(iss >> status >> link >> level)) {
    state.wifi_signal_dbm = 0;
    state.wifi_link_quality = 0.0f;
    return;
  }
  // /proc/net/wireless prints values like "54." (dot suffix). std::istringstream
  // tolerates that, but `link` is a quality counter 0..70 (driver-specific scale).
  // We normalise to 0..1 by clamping at /70.
  double q = link / 70.0;
  if (q < 0.0) q = 0.0;
  if (q > 1.0) q = 1.0;
  state.wifi_link_quality = static_cast<float>(q);
  state.wifi_signal_dbm = static_cast<int16_t>(level);

  const bool present_now = state.wifi_signal_dbm != 0;
  if (!wifi_seen_once) {
    wifi_seen_once = true;
    wifi_currently_present = present_now;
    wifi_pending_state = present_now;
    return;
  }
  const ros::Time now = ros::Time::now();
  if (present_now == wifi_currently_present) {
    wifi_pending_since = ros::Time(0, 0);
    return;
  }
  if (present_now != wifi_pending_state) {
    wifi_pending_state = present_now;
    wifi_pending_since = now;
    return;
  }
  if (wifi_pending_since.isZero() || (now - wifi_pending_since) < kWifiTransitionThreshold) {
    return;
  }
  wifi_currently_present = present_now;
  wifi_pending_since = ros::Time(0, 0);
  if (present_now) {
    open_mower::events::EventPublisher::info(
        "wifi.restored", "WiFi link restored",
        {{"signal_dbm", state.wifi_signal_dbm}, {"link_quality", state.wifi_link_quality}});
  } else {
    open_mower::events::EventPublisher::warning("wifi.lost", "WiFi link lost");
  }
}

void pose_received(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
  state.robot_pose = *msg;

  // Rate limit to 2Hz
  static ros::Time last_update{0};
  if ((msg->header.stamp - last_update).toSec() < 0.5) return;
  last_update = msg->header.stamp;

  // Refresh WLAN stats at the same rate as pose updates — cheap (one
  // /proc read per 0.5 s) and naturally rate-limited.
  update_wifi_stats();

  xbot_msgs::SensorDataDouble sensor_data;
  sensor_data.stamp = msg->header.stamp;
  sensor_data.data = msg->position_accuracy;

  auto sc_it = sensor_configs.find("om_gps_accuracy");
  if (sc_it != std::end(sensor_configs)) {
    sc_it->second.data_pub.publish(sensor_data);
  }

  // Heading accuracy is only meaningful when orientation_valid is set; the
  // backend otherwise publishes a stale or default value that would pollute
  // the chart. AbsolutePose.orientation_accuracy is in rad — convert to deg
  // to match the sensor's declared unit.
  if (msg->orientation_valid) {
    auto sc_heading_it = sensor_configs.find("om_gps_heading_accuracy");
    if (sc_heading_it != std::end(sensor_configs)) {
      xbot_msgs::SensorDataDouble heading_data;
      heading_data.stamp = msg->header.stamp;
      heading_data.data = msg->orientation_accuracy * 180.0 / M_PI;
      sc_heading_it->second.data_pub.publish(heading_data);
    }
  }
}

void power_received(const mower_msgs::Power::ConstPtr& msg) {
  // Rate limit to 2Hz
  static ros::Time last_update{0};
  if ((msg->stamp - last_update).toSec() < 0.5) return;
  last_update = msg->stamp;
  {
    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->charge_voltage_chg > 0.0 ? msg->charge_voltage_chg : msg->charge_voltage_adc;

    auto sc_it = sensor_configs.find("om_v_charge");
    if (sc_it != std::end(sensor_configs)) {
      sc_it->second.data_pub.publish(sensor_data);
    }
  }
  {
    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->battery_voltage_chg > 0.0 ? msg->battery_voltage_chg : msg->battery_voltage_adc;

    auto sc_it = sensor_configs.find("om_v_battery");
    if (sc_it != std::end(sensor_configs)) {
      sc_it->second.data_pub.publish(sensor_data);
    }
  }
  {
    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->charge_current;

    auto sc_it = sensor_configs.find("om_charge_current");
    if (sc_it != std::end(sensor_configs)) {
      sc_it->second.data_pub.publish(sensor_data);
    }
  }
  {
    xbot_msgs::SensorDataString sensor_data;
    sensor_data.stamp = msg->stamp;
    sensor_data.data = msg->charger_status;

    auto sc_it = sensor_configs.find("om_charge_state");
    if (sc_it != std::end(sensor_configs)) {
      sc_it->second.data_pub.publish(sensor_data);
    }
  }
}

void left_esc_status_received(const mower_msgs::ESCStatus::ConstPtr& msg) {
  // Rate limit to 2Hz
  static ros::Time last_update{0};
  const auto now = ros::Time::now();
  if ((now - last_update).toSec() < 0.5) return;
  last_update = now;
  {
    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = ros::Time::now();
    sensor_data.data = msg->temperature_pcb;
    auto sc_it = sensor_configs.find("om_left_esc_temp");
    if (sc_it != std::end(sensor_configs)) {
      sc_it->second.data_pub.publish(sensor_data);
    }
  }
}

void right_esc_status_received(const mower_msgs::ESCStatus::ConstPtr& msg) {
  // Rate limit to 2Hz
  static ros::Time last_update{0};
  const auto now = ros::Time::now();
  if ((now - last_update).toSec() < 0.5) return;
  last_update = now;
  {
    xbot_msgs::SensorDataDouble sensor_data;
    sensor_data.stamp = ros::Time::now();
    sensor_data.data = msg->temperature_pcb;
    auto sc_it = sensor_configs.find("om_right_esc_temp");
    if (sc_it != std::end(sensor_configs)) {
      sc_it->second.data_pub.publish(sensor_data);
    }
  }
}

void set_limits_battery_v(SensorConfig& sensor_config) {
  sensor_config.si.lower_critical_value = power_config.battery_critical_voltage;
  sensor_config.si.min_value = power_config.battery_empty_voltage;
  sensor_config.si.max_value = power_config.battery_full_voltage;
  sensor_config.si.upper_critical_value = power_config.battery_critical_high_voltage;
}

void set_limits_charge_v(SensorConfig& sensor_config) {
  sensor_config.si.upper_critical_value = power_config.charge_critical_high_voltage;
}

void set_limits_charge_current(SensorConfig& sensor_config) {
  sensor_config.si.upper_critical_value = power_config.charge_critical_high_current;
}

void set_limits_esc_temp(SensorConfig& sensor_config) {
  sensor_config.si.max_value = paramNh->param(sensor_config.param_path + "/max_pcb_temp", 0);
}

void set_limits_mow_motor_current(SensorConfig& sensor_config) {
  sensor_config.si.upper_critical_value = paramNh->param(sensor_config.param_path + "/motor_current_limit", 0.0f);
}

void set_limits_mow_motor_rpm(SensorConfig& sensor_config) {
  // Use the labeled YF-C500 mow motor value (3800 rpm) for default threshold estimations...
  sensor_config.si.lower_critical_value = paramNh->param(sensor_config.param_path + "/min_motor_rpm_critical", 2300);
  sensor_config.si.min_value = paramNh->param(sensor_config.param_path + "/min_motor_rpm", 2800);
  sensor_config.si.max_value = paramNh->param(sensor_config.param_path + "/max_motor_rpm", 3800);
}

void set_limits_mow_motor_temp(SensorConfig& sensor_config) {
  // mower_config settings have precedence before xesc param file because user editable
  sensor_config.si.max_value = mower_logic_config.motor_hot_temperature;
  sensor_config.si.min_value = mower_logic_config.motor_cold_temperature;
}

void registerSensors() {
  for (auto& sc_pair : sensor_configs) {
    if (sc_pair.second.existCB && !sc_pair.second.existCB()) {
      ROS_INFO_STREAM("Skipped monitoring of sensor " << sc_pair.first);
      continue;
    }

    sc_pair.second.si.sensor_id = sc_pair.first;
    sc_pair.second.si.sensor_name = sc_pair.second.name;

    sc_pair.second.si.unit = sc_pair.second.unit;
    sc_pair.second.si.value_type = sc_pair.second.sensor_type;
    sc_pair.second.si.value_description = sc_pair.second.value_desc;

    // Set sensor threshold values
    if (sc_pair.second.setSensorLimitsCB) sc_pair.second.setSensorLimitsCB(sc_pair.second);

    // Set has* sensor infos
    // FIXME: "has_min_max" is somehow confusing/misstakeable.
    // From the logic point of view, has_min_max should only be set if it has min as well as max limits,
    // but then it's somehow useless because i.e. for temperatures, we don't have a reasonable min value.
    // At least it doesn't make much sense to show i.e. a gauge from -15 to 80°C.
    // If has_min_max get set if min OR max is set, then it's useless again, because then we need to check
    // min as well as max for a limit value.
    // In my opinion, we can drop all has_* settings (except has_motor_temp) and let decide the view logic how to handle
    // the limits
    if (sc_pair.second.si.min_value && sc_pair.second.si.max_value) sc_pair.second.si.has_min_max = true;
    if (sc_pair.second.si.lower_critical_value) sc_pair.second.si.has_critical_low = true;
    if (sc_pair.second.si.upper_critical_value) sc_pair.second.si.has_critical_high = true;

    sc_pair.second.si_pub =
        n->advertise<xbot_msgs::SensorInfo>("xbot_monitoring/sensors/" + sc_pair.first + "/info", 1, true);
    switch (sc_pair.second.si.value_type) {
      case xbot_msgs::SensorInfo::TYPE_DOUBLE:
        sc_pair.second.data_pub =
            n->advertise<xbot_msgs::SensorDataDouble>("xbot_monitoring/sensors/" + sc_pair.first + "/data", 10);
        break;
      case xbot_msgs::SensorInfo::TYPE_STRING:
        sc_pair.second.data_pub =
            n->advertise<xbot_msgs::SensorDataString>("xbot_monitoring/sensors/" + sc_pair.first + "/data", 10);
        break;
      default: ROS_ERROR_STREAM("Invalid Sensor Data Type: " << (int)sc_pair.second.si.value_type);
    }
    sc_pair.second.si_pub.publish(sc_pair.second.si);
  }
}

void logicReconfigCB(const mower_logic::MowerLogicConfig& config) {
  ROS_INFO_STREAM("Monitoring received new mower_logic config");
  mower_logic_config = config;

  registerSensors();
}

void powerReconfigCB(const ll::PowerConfig& config) {
  ROS_INFO_STREAM("Monitoring received new power config");
  power_config = config;

  registerSensors();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "monitoring");

  n = new ros::NodeHandle();
  paramNh = new ros::NodeHandle("/mower_comms");
  open_mower::events::EventPublisher::init(*n, "monitoring");
  ros::NodeHandle logicParamNh{"/mower_logic"};
  ros::NodeHandle powerParamNh{"/ll/services/power"};

  mower_logic_config = mower_logic::MowerLogicConfig::__getDefault__();
  mower_logic_config.__fromServer__(logicParamNh);
  power_config = ll::PowerConfig::__getDefault__();
  power_config.__fromServer__(powerParamNh);
  logicReconfigClient = new dynamic_reconfigure::Client<mower_logic::MowerLogicConfig>("/mower_logic", logicReconfigCB);
  powerReconfigClient = new dynamic_reconfigure::Client<ll::PowerConfig>("/ll/services/power", powerReconfigCB);

  registerSensors();

  ros::Subscriber state_sub = n->subscribe("mower_logic/current_state", 10, high_level_status);
  ros::Subscriber status_state_subscriber = n->subscribe("/ll/mower_status", 10, status_received);
  ros::Subscriber power_state_subscriber = n->subscribe("/ll/power", 10, power_received);
  ros::Subscriber left_esc_status_state_subscriber =
      n->subscribe("/ll/diff_drive/left_esc_status", 10, left_esc_status_received);
  ros::Subscriber right_esc_status_state_subscriber =
      n->subscribe("/ll/diff_drive/right_esc_status", 10, right_esc_status_received);
  ros::Subscriber pose_state_subscriber = n->subscribe("/xbot_positioning/xb_pose", 10, pose_received);
  // mower_comms_v2 publishes detailed GPS fix metadata at this topic. v1's
  // driver_gps_node publishes the same shape on /<gps_node>/gps_status if/when
  // we wire that up — for now only v2 is in use on the actual hardware.
  ros::Subscriber gps_status_subscriber = n->subscribe("/ll/position/gps_status", 10, gps_status_received);

  state_pub = n->advertise<xbot_msgs::RobotState>("xbot_monitoring/robot_state", 10);

  ros::spin();

  return 0;
}
