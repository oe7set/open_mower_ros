// In-memory ring buffer of recent SensorDataDouble samples per sensor id.
//
// xbot_monitoring is the single subscriber/publisher of the sensor topics it
// bridges to MQTT. We tap that path to keep ~1 hour of numeric history for
// every known sensor so the frontend can render charts immediately on
// connect (instead of waiting for fresh live samples to dribble in). The
// buffer is purely process-local — it is intentionally not persisted: the
// user explicitly wants a "live" feel, and disk I/O on the CM4 is a hot
// resource we are unwilling to spend on transient telemetry.
//
// JSON shape on the wire (single-sensor):
//   { "sensor_id": "om_v_battery",
//     "samples":  [ { "ts_ms": 1715..., "value": 27.85 }, ... ] }
//
// Bulk shape:
//   { "sensors": { "om_v_battery": [ {ts_ms, value}, ... ], ... } }

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <unordered_map>

namespace xbot_monitoring::sensors_history {

// 1 h at 2 Hz update rate covers our worst-case (pose-driven) sensors.
// Sensors that update more slowly (HighLevelStatus ~1 Hz) simply use less of
// the buffer. Memory budget: 12 sensors * 7200 * 16 B ≈ 1.4 MB worst-case
// for the binary store, plus JSON overhead at serialization time.
constexpr std::size_t kSamplesPerSensor = 7200;

struct Sample {
  uint64_t ts_ms = 0;  // wall-clock epoch milliseconds
  double value = 0.0;
};

class SensorHistory {
 public:
  explicit SensorHistory(std::size_t capacity_per_sensor = kSamplesPerSensor);

  // Append a numeric sample. ts_ms is wall-clock epoch milliseconds — caller
  // is expected to derive it from ros::Time::now() so the ordering matches
  // the live MQTT stream consumed by the frontend.
  void push(const std::string& sensor_id, double value, uint64_t ts_ms);

  // Return samples for a single sensor, oldest→newest. `since_ts` (when set)
  // filters strictly newer entries. `limit` caps the number of returned
  // samples after filtering and is clamped to [1, capacity].
  nlohmann::json list_json(const std::string& sensor_id,
                           std::optional<uint64_t> since_ts,
                           std::size_t limit) const;

  // Return samples for every sensor that has at least one entry. Same
  // filtering semantics as list_json. Used by the frontend on app boot for
  // a single-roundtrip warm-up.
  nlohmann::json list_all_json(std::optional<uint64_t> since_ts,
                                std::size_t limit) const;

 private:
  static nlohmann::json samples_to_json(const std::deque<Sample>& samples,
                                        std::optional<uint64_t> since_ts,
                                        std::size_t limit);

  std::size_t capacity_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::deque<Sample>> store_;
};

}  // namespace xbot_monitoring::sensors_history
