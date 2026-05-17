#include "event_publisher/event_publisher.hpp"

#include <ros/ros.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <chrono>
#include <mutex>

namespace open_mower::events {

namespace {

std::mutex g_mutex;
ros::Publisher g_publisher;
std::string g_source;
bool g_initialized = false;
bool g_warned_uninitialized = false;

uint64_t now_epoch_ms() {
  using namespace std::chrono;
  return static_cast<uint64_t>(duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count());
}

std::string make_uuid() {
  static thread_local boost::uuids::random_generator gen;
  return boost::uuids::to_string(gen());
}

}  // namespace

void EventPublisher::init(ros::NodeHandle& nh, const std::string& source) {
  std::lock_guard<std::mutex> lock(g_mutex);
  // Topic is global on purpose — every producer publishes onto the same bus
  // so xbot_monitoring (the single subscriber) sees them all.
  g_publisher = nh.advertise<xbot_msgs::Event>("/events", 20, /*latch=*/false);
  g_source = source;
  g_initialized = true;
}

void EventPublisher::emit(uint8_t severity, const std::string& type, const std::string& summary,
                          const nlohmann::json& details) {
  std::lock_guard<std::mutex> lock(g_mutex);
  if (!g_initialized) {
    if (!g_warned_uninitialized) {
      ROS_WARN("EventPublisher::emit called before init(); event '%s' dropped", type.c_str());
      g_warned_uninitialized = true;
    }
    return;
  }

  xbot_msgs::Event msg;
  msg.header.stamp = ros::Time::now();
  msg.id = make_uuid();
  msg.ts_ms = now_epoch_ms();
  msg.severity = severity;
  msg.type = type;
  msg.source = g_source;
  msg.summary = summary;
  msg.details_json = details.empty() ? std::string() : details.dump();

  g_publisher.publish(msg);
}

void EventPublisher::info(const std::string& type, const std::string& summary, const nlohmann::json& details) {
  emit(xbot_msgs::Event::SEVERITY_INFO, type, summary, details);
}

void EventPublisher::warning(const std::string& type, const std::string& summary, const nlohmann::json& details) {
  emit(xbot_msgs::Event::SEVERITY_WARNING, type, summary, details);
}

void EventPublisher::error(const std::string& type, const std::string& summary, const nlohmann::json& details) {
  emit(xbot_msgs::Event::SEVERITY_ERROR, type, summary, details);
}

void EventPublisher::critical(const std::string& type, const std::string& summary, const nlohmann::json& details) {
  emit(xbot_msgs::Event::SEVERITY_CRITICAL, type, summary, details);
}

}  // namespace open_mower::events
