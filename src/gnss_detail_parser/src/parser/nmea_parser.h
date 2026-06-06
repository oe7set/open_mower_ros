#ifndef GNSS_DETAIL_PARSER_NMEA_PARSER_H
#define GNSS_DETAIL_PARSER_NMEA_PARSER_H

#include <array>
#include <cstdint>

#include "gnss_detail_parser/gnss_detail_parser.h"

namespace gnss_detail_parser {

// NMEA + Unicore proprietary ASCII parser. Ported from the v2 firmware
// NmeaGpsDriver so the produced GnssDetail is byte-for-byte what the firmware
// used to publish. Line framing, minmea usage and the Unicore field offsets are
// all preserved. The epoch boundary is GGA, which the receiver emits once per
// solution; the accumulated GSV/GSA/Unicore detail is published there.
class NmeaParser : public Parser {
 public:
  void ProcessBytes(const uint8_t* data, size_t len) override;
  void Reset() override;
  std::vector<std::vector<uint8_t>> StartupCommands() const override;

 private:
  bool ProcessLine(const char* line);
  bool ProcessUnicoreLine(const char* line);
  bool ParseHDT(const char* line);
  void ProcessGsv(const char* line);
  void AccumulateGsaUsed(const char* line, const int* sats);
  void CommitGsv();

  // Maximum per-signal satellite records carried, matching the firmware.
  static constexpr size_t kMaxSats = 60;

  char line_[512]{};
  size_t line_len_ = 0;

  int fix_quality_ = 0;

  // State accumulated across the sentences of the current epoch.
  GnssDetailState state_;

  // Scratch for the in-progress epoch's GSV satellites.
  std::array<SatInfo, kMaxSats> gsv_scratch_{};
  uint8_t gsv_fill_ = 0;

  // Used-in-solution (gnss_id, sv_id) pairs from GSA this epoch.
  struct UsedSat {
    uint8_t gnss_id;
    uint8_t sv_id;
  };
  std::array<UsedSat, kMaxSats> gsa_used_{};
  uint8_t gsa_used_fill_ = 0;
};

}  // namespace gnss_detail_parser

#endif  // GNSS_DETAIL_PARSER_NMEA_PARSER_H
