#ifndef GNSS_DETAIL_PARSER_UBX_PARSER_H
#define GNSS_DETAIL_PARSER_UBX_PARSER_H

#include <array>
#include <cstdint>

#include "gnss_detail_parser/gnss_detail_parser.h"
#include "ubx_datatypes.h"

namespace gnss_detail_parser {

// UBX binary parser. Ported from the v2 firmware UbxGpsDriver: framing + the
// NAV-PVT / NAV-SAT / NAV-SIG / NAV-DOP handlers that fill the GnssDetail
// snapshot. The epoch boundary is NAV-PVT (one per nav solution); NAV-SAT/SIG
// build the satellite list which NAV-PVT then publishes.
class UbxParser : public Parser {
 public:
  void ProcessBytes(const uint8_t* data, size_t len) override;
  void Reset() override;
  std::vector<std::vector<uint8_t>> StartupCommands() const override;

 private:
  static constexpr size_t kMaxSats = 60;

  size_t ProcessRingBuffer(const uint8_t* buffer, size_t len);
  bool ValidateChecksum(const uint8_t* packet, size_t size) const;
  static void CalculateChecksum(const uint8_t* packet, size_t size, uint8_t& ck_a, uint8_t& ck_b);
  void ProcessUbxPacket(const uint8_t* data, size_t size);
  void HandleNavPvt(const UbxNavPvt* msg);
  void HandleNavSat(const uint8_t* payload, size_t size);
  void HandleNavSig(const uint8_t* payload, size_t size);
  void HandleNavDop(const UbxNavDop* msg);
  void RebuildSatelliteState();
  std::vector<uint8_t> BuildValset(const uint32_t* keys, size_t num_keys) const;

  uint8_t gbuffer_[512]{};
  size_t gbuffer_fill_ = 0;
  bool found_header_ = false;

  GnssDetailState state_;

  // Cached NAV-SAT rows so NAV-SIG can attach sky position by (gnss, sv).
  std::array<UbxNavSatSv, kMaxSats> nav_sat_{};
  uint8_t nav_sat_count_ = 0;
};

}  // namespace gnss_detail_parser

#endif  // GNSS_DETAIL_PARSER_UBX_PARSER_H
