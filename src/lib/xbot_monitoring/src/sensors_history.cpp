#include "sensors_history.h"

#include <algorithm>

using json = nlohmann::json;

namespace xbot_monitoring::sensors_history {

SensorHistory::SensorHistory(std::size_t capacity_per_sensor) : capacity_(capacity_per_sensor) {}

void SensorHistory::push(const std::string& sensor_id, double value, uint64_t ts_ms) {
  if (sensor_id.empty()) return;
  std::lock_guard<std::mutex> lk(mutex_);
  auto& dq = store_[sensor_id];
  dq.push_back(Sample{ts_ms, value});
  while (dq.size() > capacity_) {
    dq.pop_front();
  }
}

json SensorHistory::samples_to_json(const std::deque<Sample>& samples,
                                    std::optional<uint64_t> since_ts,
                                    std::size_t limit) {
  json out = json::array();
  if (samples.empty() || limit == 0) return out;

  // Walk newest→oldest, then reverse so the result reads chronologically.
  // This way `limit` keeps the most-recent N samples — the natural fit for a
  // trailing chart window.
  std::vector<Sample> picked;
  picked.reserve(std::min(limit, samples.size()));
  for (auto it = samples.rbegin(); it != samples.rend() && picked.size() < limit; ++it) {
    if (since_ts && it->ts_ms <= *since_ts) break;
    picked.push_back(*it);
  }
  for (auto it = picked.rbegin(); it != picked.rend(); ++it) {
    out.push_back({{"ts_ms", it->ts_ms}, {"value", it->value}});
  }
  return out;
}

json SensorHistory::list_json(const std::string& sensor_id,
                              std::optional<uint64_t> since_ts,
                              std::size_t limit) const {
  std::size_t effective_limit = std::clamp<std::size_t>(limit, 1, capacity_);
  json samples = json::array();
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = store_.find(sensor_id);
    if (it != store_.end()) {
      samples = samples_to_json(it->second, since_ts, effective_limit);
    }
  }
  return json{{"sensor_id", sensor_id}, {"samples", std::move(samples)}};
}

json SensorHistory::list_all_json(std::optional<uint64_t> since_ts, std::size_t limit) const {
  std::size_t effective_limit = std::clamp<std::size_t>(limit, 1, capacity_);
  json sensors = json::object();
  {
    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& [sensor_id, dq] : store_) {
      json entry = samples_to_json(dq, since_ts, effective_limit);
      if (!entry.empty()) {
        sensors[sensor_id] = std::move(entry);
      }
    }
  }
  return json{{"sensors", std::move(sensors)}};
}

}  // namespace xbot_monitoring::sensors_history
