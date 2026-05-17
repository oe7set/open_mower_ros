// Notification / event store backing the events.* RPC surface.
//
// The store is an in-memory ring buffer of the last N events received on the
// `/events` ROS topic. xbot_monitoring is the single subscriber and the only
// writer. Snapshots are persisted atomically to disk so history survives node
// restarts. JSON shape on the wire:
//
//   {
//     "id": "uuid", "ts_ms": 17051234..., "severity": "info|warning|error|critical",
//     "type": "mowing.started", "source": "mower_logic",
//     "summary": "Mowing started", "details": { ... } | null,
//     "acked": false
//   }

#pragma once

#include <xbot_msgs/Event.h>

#include <chrono>
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace xbot_monitoring::events_io {

constexpr size_t kDefaultCapacity = 500;
constexpr int kDiskFormatVersion = 1;

struct Event {
  std::string id;
  uint64_t ts_ms = 0;
  uint8_t severity = 0;  // matches xbot_msgs::Event::SEVERITY_*
  std::string type;
  std::string source;
  std::string summary;
  // Parsed details (object). Empty object = no payload.
  nlohmann::json details = nlohmann::json::object();
  bool acked = false;
};

struct ListFilter {
  std::optional<uint64_t> since_ts;
  std::optional<uint8_t> severity_min;
  std::vector<std::string> types;  // empty = no filter
  std::optional<size_t> limit;
};

class EventStore {
 public:
  explicit EventStore(size_t capacity = kDefaultCapacity);

  // Load persisted events from disk. Missing file is not an error — the store
  // simply starts empty. Malformed files are logged and ignored.
  void load(const std::string& path);

  // Append an event from the ROS message. Returns the canonical JSON object
  // for callers that want to immediately re-publish. Trims to capacity.
  nlohmann::json add(const xbot_msgs::Event& msg);

  // Mark a single event acked. No-op if id is unknown.
  bool ack(const std::string& id);

  // Mark every event acked.
  void ack_all();

  // Drop every event. Caller is responsible for persisting the empty state.
  void clear();

  // Filter a list of events as JSON, newest first. `limit` (when set) caps the
  // number of returned entries.
  nlohmann::json list_json(const ListFilter& filter) const;

  // Build the retained events/json snapshot — full buffer, newest first, plus
  // unread count.
  nlohmann::json snapshot_json() const;

  // Number of unacked events in the buffer.
  size_t unread_count() const;

  // Atomically persist the current buffer to `path`. Caller may invoke from
  // a background thread.
  void persist(const std::string& path) const;

 private:
  static nlohmann::json event_to_json(const Event& e);
  static const char* severity_to_string(uint8_t s);
  static uint8_t severity_from_string(const std::string& s);

  size_t capacity_;
  mutable std::mutex mutex_;
  std::deque<Event> events_;
};

}  // namespace xbot_monitoring::events_io
