#ifndef GNSS_DETAIL_PARSER_H
#define GNSS_DETAIL_PARSER_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace gnss_detail_parser {

// Frequency band normalization, identical to the firmware GpsState::BandFromSignal
// (u-blox UBX-NAV-SIG sigId convention, also reused for the NMEA 4.11 signalId).
// 1 = L1/E1/B1, 2 = L2/B2I, 5 = L5/E5/B2a, 0 = unknown.
uint8_t BandFromSignal(uint8_t gnss_id, uint8_t sig_id);

// A single tracked GNSS signal. One satellite may appear multiple times when it
// is tracked on more than one frequency band. Mirrors xbot_msgs/Satellite.
struct SatInfo {
  uint8_t gnss_id = 255;
  uint8_t sv_id = 0;
  uint8_t cn0 = 0;
  uint8_t band = 0;
  int16_t elevation = -128;  // degrees, -128 = unknown
  int16_t azimuth = -1;      // degrees 0..360, -1 = unknown
  bool used = false;
  bool healthy = false;
};

// GNSS-page diagnostic snapshot, accumulated over one receiver epoch. The field
// set mirrors xbot_msgs/GnssDetail so the ROS node maps it 1:1. Defaults match
// the firmware's "not reported" sentinels so the app renders identically.
struct GnssDetailState {
  std::vector<SatInfo> satellites;
  uint16_t sats_visible = 0;
  uint16_t sats_used = 0;

  float gdop = 0, pdop = 0, hdop = 0, vdop = 0, tdop = 0;

  // Fix quality on the GpsStatus scale: 0 none, 1 2D, 2 3D, 3 DGPS, 4 RTK float,
  // 5 RTK fixed. rtk_type: 0 none, 1 float, 2 fixed.
  uint8_t fix_type = 0;
  uint8_t rtk_type = 0;

  double lat = 0, lon = 0;
  float height = 0;
  float h_acc = 0, v_acc = 0;

  float vel_e = 0, vel_n = 0, vel_u = 0;

  float vehicle_heading = 0;  // rad
  float motion_heading = 0;   // rad

  float correction_age = 0;
  float baseline_len = 0;
  float heading_accuracy = 0;  // degrees
  uint8_t solution_status = 255;
  float elevation_cutoff = -1;
  std::vector<int8_t> antenna_agc;
  std::vector<uint8_t> jamming;
};

// Abstract byte-stream parser. Concrete protocols (NMEA, UBX) accumulate state
// and invoke the epoch callback once per receiver epoch (the boundary where a
// coherent GnssDetail snapshot is ready: GGA for NMEA, NAV-PVT for UBX).
class Parser {
 public:
  using EpochCallback = std::function<void(const GnssDetailState&)>;

  virtual ~Parser() = default;

  void SetEpochCallback(EpochCallback cb) {
    epoch_callback_ = std::move(cb);
  }

  // Feed received bytes. Calls the epoch callback when a full epoch is complete.
  virtual void ProcessBytes(const uint8_t* data, size_t len) = 0;

  // Reset on (re)connect so a partial frame from a dropped link is discarded.
  virtual void Reset() = 0;

  // Receiver-configuration commands to push on connect so the detail messages
  // we parse actually get emitted (the firmware no longer requests these).
  // Returned as raw bytes ready to write back over the same link.
  virtual std::vector<std::vector<uint8_t>> StartupCommands() const = 0;

 protected:
  void EmitEpoch(const GnssDetailState& state) {
    if (epoch_callback_) epoch_callback_(state);
  }

  EpochCallback epoch_callback_;
};

}  // namespace gnss_detail_parser

#endif  // GNSS_DETAIL_PARSER_H
