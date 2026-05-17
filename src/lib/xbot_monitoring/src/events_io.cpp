#include "events_io.h"

#include <ros/ros.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace xbot_monitoring::events_io {

namespace {

// Atomic write: <path>.tmp + rename. Mirrors config_io::write_text_file_atomic
// so the events.json file behaves identically to mower_config.sh under
// crashes mid-write.
void write_text_file_atomic(const std::string& path, const std::string& content) {
  fs::path p(path);
  if (p.has_parent_path()) {
    std::error_code ec;
    fs::create_directories(p.parent_path(), ec);
    // Ignore failure; the open below will surface a clearer error.
  }
  std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
    if (!f) {
      throw std::runtime_error("Could not open temp file for write: " + tmp);
    }
    f << content;
    if (!f) {
      throw std::runtime_error("Write failed: " + tmp);
    }
  }
  std::error_code ec;
  fs::rename(tmp, path, ec);
  if (ec) {
    throw std::runtime_error("Atomic rename failed: " + ec.message());
  }
}

}  // namespace

EventStore::EventStore(size_t capacity) : capacity_(capacity) {}

const char* EventStore::severity_to_string(uint8_t s) {
  switch (s) {
    case xbot_msgs::Event::SEVERITY_INFO: return "info";
    case xbot_msgs::Event::SEVERITY_WARNING: return "warning";
    case xbot_msgs::Event::SEVERITY_ERROR: return "error";
    case xbot_msgs::Event::SEVERITY_CRITICAL: return "critical";
    default: return "info";
  }
}

uint8_t EventStore::severity_from_string(const std::string& s) {
  if (s == "warning") return xbot_msgs::Event::SEVERITY_WARNING;
  if (s == "error") return xbot_msgs::Event::SEVERITY_ERROR;
  if (s == "critical") return xbot_msgs::Event::SEVERITY_CRITICAL;
  return xbot_msgs::Event::SEVERITY_INFO;
}

json EventStore::event_to_json(const Event& e) {
  json j;
  j["id"] = e.id;
  j["ts_ms"] = e.ts_ms;
  j["severity"] = severity_to_string(e.severity);
  j["type"] = e.type;
  j["source"] = e.source;
  j["summary"] = e.summary;
  // Always emit a details object so the frontend never has to distinguish
  // "missing" from "empty".
  j["details"] = e.details.empty() ? json::object() : e.details;
  j["acked"] = e.acked;
  return j;
}

void EventStore::load(const std::string& path) {
  std::ifstream f(path);
  if (!f) {
    // Missing file → start empty; not an error.
    return;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  json doc;
  try {
    doc = json::parse(ss.str());
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("events_io: failed to parse " << path << ": " << e.what()
                                                  << "; starting empty.");
    return;
  }

  if (!doc.is_object() || !doc.contains("events") || !doc["events"].is_array()) {
    ROS_WARN_STREAM("events_io: " << path << " has unexpected shape; starting empty.");
    return;
  }

  std::lock_guard<std::mutex> lk(mutex_);
  events_.clear();
  for (const auto& entry : doc["events"]) {
    if (!entry.is_object()) continue;
    Event e;
    e.id = entry.value("id", std::string());
    e.ts_ms = entry.value("ts_ms", uint64_t{0});
    if (entry.contains("severity") && entry["severity"].is_string()) {
      e.severity = severity_from_string(entry["severity"].get<std::string>());
    }
    e.type = entry.value("type", std::string());
    e.source = entry.value("source", std::string());
    e.summary = entry.value("summary", std::string());
    if (entry.contains("details") && entry["details"].is_object()) {
      e.details = entry["details"];
    }
    e.acked = entry.value("acked", false);
    events_.push_back(std::move(e));
  }
  while (events_.size() > capacity_) {
    events_.pop_front();
  }
}

json EventStore::add(const xbot_msgs::Event& msg) {
  Event e;
  e.id = msg.id;
  e.ts_ms = msg.ts_ms;
  e.severity = msg.severity;
  e.type = msg.type;
  e.source = msg.source;
  e.summary = msg.summary;
  if (!msg.details_json.empty()) {
    try {
      auto parsed = json::parse(msg.details_json);
      if (parsed.is_object()) {
        e.details = std::move(parsed);
      }
    } catch (const std::exception& parse_err) {
      ROS_WARN_STREAM("events_io: failed to parse details_json for event "
                      << msg.type << ": " << parse_err.what());
    }
  }

  json view;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    events_.push_back(e);
    while (events_.size() > capacity_) {
      events_.pop_front();
    }
    view = event_to_json(events_.back());
  }
  return view;
}

bool EventStore::ack(const std::string& id) {
  std::lock_guard<std::mutex> lk(mutex_);
  for (auto& e : events_) {
    if (e.id == id) {
      e.acked = true;
      return true;
    }
  }
  return false;
}

void EventStore::ack_all() {
  std::lock_guard<std::mutex> lk(mutex_);
  for (auto& e : events_) {
    e.acked = true;
  }
}

void EventStore::clear() {
  std::lock_guard<std::mutex> lk(mutex_);
  events_.clear();
}

json EventStore::list_json(const ListFilter& filter) const {
  json out = json::array();
  std::lock_guard<std::mutex> lk(mutex_);
  // Iterate newest-first so `limit` truncates the oldest side.
  size_t emitted = 0;
  for (auto it = events_.rbegin(); it != events_.rend(); ++it) {
    if (filter.since_ts && it->ts_ms < *filter.since_ts) continue;
    if (filter.severity_min && it->severity < *filter.severity_min) continue;
    if (!filter.types.empty()) {
      if (std::find(filter.types.begin(), filter.types.end(), it->type) == filter.types.end()) {
        continue;
      }
    }
    out.push_back(event_to_json(*it));
    ++emitted;
    if (filter.limit && emitted >= *filter.limit) break;
  }
  return out;
}

json EventStore::snapshot_json() const {
  json events_arr = json::array();
  size_t unread = 0;
  std::lock_guard<std::mutex> lk(mutex_);
  for (auto it = events_.rbegin(); it != events_.rend(); ++it) {
    events_arr.push_back(event_to_json(*it));
    if (!it->acked) ++unread;
  }
  return json{
      {"events", events_arr},
      {"unread", unread},
  };
}

size_t EventStore::unread_count() const {
  std::lock_guard<std::mutex> lk(mutex_);
  size_t n = 0;
  for (const auto& e : events_) {
    if (!e.acked) ++n;
  }
  return n;
}

void EventStore::persist(const std::string& path) const {
  json doc;
  doc["version"] = kDiskFormatVersion;
  json events_arr = json::array();
  {
    std::lock_guard<std::mutex> lk(mutex_);
    // Persist oldest-first so the on-disk file reads naturally; load() is
    // order-agnostic but a chronological file is friendlier for grep.
    for (const auto& e : events_) {
      events_arr.push_back(event_to_json(e));
    }
  }
  doc["events"] = std::move(events_arr);
  write_text_file_atomic(path, doc.dump(2));
}

}  // namespace xbot_monitoring::events_io
