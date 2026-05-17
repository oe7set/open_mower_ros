// Single-header surface of the EventPublisher helper.
//
// Producers call EventPublisher::init(nh, "<source>") once at node startup
// and then EventPublisher::emit(severity, type, summary, details) at every
// lifecycle transition. The helper attaches a UUID, current ROS wall time
// and the configured source field, then publishes on /events with queue 20,
// latched=false. xbot_monitoring is the single subscriber that persists and
// fans the events out to MQTT.

#pragma once

#include <ros/ros.h>
#include <xbot_msgs/Event.h>

#include <nlohmann/json.hpp>
#include <string>

namespace open_mower::events {

class EventPublisher {
 public:
  // Initialise the singleton publisher. Must be called once per node before
  // emit(). Calling again with a different node handle replaces the
  // publisher; calling again from the same node is a no-op.
  static void init(ros::NodeHandle& nh, const std::string& source);

  // Emit an event. Safe to call before init() — the call is silently dropped
  // and a warning logged once. `details` is serialised to JSON; pass an empty
  // object (the default) to omit the payload.
  static void emit(uint8_t severity, const std::string& type, const std::string& summary,
                   const nlohmann::json& details = nlohmann::json::object());

  // Convenience wrappers — same as emit() with the matching severity.
  static void info(const std::string& type, const std::string& summary,
                   const nlohmann::json& details = nlohmann::json::object());
  static void warning(const std::string& type, const std::string& summary,
                      const nlohmann::json& details = nlohmann::json::object());
  static void error(const std::string& type, const std::string& summary,
                    const nlohmann::json& details = nlohmann::json::object());
  static void critical(const std::string& type, const std::string& summary,
                       const nlohmann::json& details = nlohmann::json::object());
};

}  // namespace open_mower::events
