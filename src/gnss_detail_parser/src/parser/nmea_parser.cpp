#include "nmea_parser.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "minmea.h"

namespace gnss_detail_parser {

// --- band / id helpers (ported verbatim from the firmware) --------------------

uint8_t BandFromSignal(uint8_t gnss_id, uint8_t sig_id) {
  // GnssId u-blox convention: 0 GPS, 1 SBAS, 2 Galileo, 3 BeiDou, 5 QZSS, 6 GLONASS.
  switch (gnss_id) {
    case 0:  // GPS
      return sig_id == 0 ? 1 : (sig_id == 3 || sig_id == 4) ? 2 : (sig_id == 6 || sig_id == 7) ? 5 : 0;
    case 1:  // SBAS
      return 1;
    case 2:  // Galileo
      return (sig_id == 0 || sig_id == 1) ? 1 : (sig_id == 3 || sig_id == 4 || sig_id == 5 || sig_id == 6) ? 5 : 0;
    case 3:  // BeiDou
      return (sig_id == 0 || sig_id == 1) ? 1 : (sig_id == 2 || sig_id == 3) ? 2 : (sig_id == 5 || sig_id == 7) ? 5 : 0;
    case 5:  // QZSS
      return (sig_id == 0 || sig_id == 1) ? 1 : (sig_id == 4 || sig_id == 5) ? 2 : (sig_id == 8 || sig_id == 9) ? 5 : 0;
    case 6:  // GLONASS
      return sig_id == 0 ? 1 : sig_id == 2 ? 2 : 0;
    default: return 0;
  }
}

namespace {

constexpr uint8_t GNSS_GPS = 0;
constexpr uint8_t GNSS_SBAS = 1;
constexpr uint8_t GNSS_GALILEO = 2;
constexpr uint8_t GNSS_BEIDOU = 3;
constexpr uint8_t GNSS_QZSS = 5;
constexpr uint8_t GNSS_GLONASS = 6;
constexpr uint8_t GNSS_UNKNOWN = 255;

// Map an NMEA talker id (the two characters after '$') to a GnssId.
uint8_t GnssIdFromTalker(const char* line) {
  if (line[0] != '$') return GNSS_UNKNOWN;
  const char a = line[1];
  const char b = line[2];
  if (a == 'G') {
    switch (b) {
      case 'P': return GNSS_GPS;
      case 'L': return GNSS_GLONASS;
      case 'A': return GNSS_GALILEO;
      case 'B': return GNSS_BEIDOU;
      case 'Q': return GNSS_QZSS;
      default: return GNSS_UNKNOWN;
    }
  }
  if (a == 'B' && b == 'D') return GNSS_BEIDOU;
  if (a == 'Q' && b == 'Z') return GNSS_QZSS;
  return GNSS_UNKNOWN;
}

// Map the NMEA 4.11 GSA/GSV systemId to a GnssId.
uint8_t GnssIdFromSystemId(int system_id) {
  switch (system_id) {
    case 1: return GNSS_GPS;
    case 2: return GNSS_GLONASS;
    case 3: return GNSS_GALILEO;
    case 4: return GNSS_BEIDOU;
    case 5: return GNSS_QZSS;
    default: return GNSS_UNKNOWN;
  }
}

// Extract the NMEA 4.11 trailing signalId from a GSV sentence (last field before
// the '*' checksum). Returns 0 ("unknown") when absent (NMEA < 4.10).
uint8_t SignalIdFromGsv(const char* line) {
  const char* star = strchr(line, '*');
  if (star == nullptr) return 0;
  const char* p = star;
  while (p > line && *(p - 1) != ',') p--;
  if (p == line || *(p - 1) != ',') return 0;
  if (p == star) return 0;  // empty field
  uint8_t value = 0;
  for (const char* c = p; c < star; c++) {
    if (*c < '0' || *c > '9') return 0;
    value = static_cast<uint8_t>(value * 10 + (*c - '0'));
  }
  return value;
}

// Map a Unicore solution/position-type token to the solution_status enum
// (0 none, 1 single, 2 DGPS, 3 float, 4 fixed; 255 = not reported).
uint8_t UnicoreSolutionStatus(const char* token) {
  if (strstr(token, "NARROW_INT") || strstr(token, "WIDE_INT") || strstr(token, "L1_INT")) return 4;
  if (strstr(token, "FLOAT")) return 3;
  if (strstr(token, "PSRDIFF") || strstr(token, "SBAS")) return 2;
  if (strstr(token, "SINGLE") || strstr(token, "FIXEDPOS") || strstr(token, "FIXEDHEIGHT") || strstr(token, "DOPPLER"))
    return 1;
  if (strstr(token, "NONE") || strstr(token, "INSUFFICIENT") || strstr(token, "NO_CONVERGENCE")) return 0;
  return 255;
}

// Copy the Nth comma-delimited field of `body` (0-based) into `out`. Stops at
// ',' '*' or end-of-string. Returns false when the field is missing/empty.
bool UnicoreField(const char* body, int idx, char* out, size_t out_len) {
  const char* p = body;
  for (int i = 0; i < idx && p; i++) {
    p = strchr(p, ',');
    if (p) p++;
  }
  if (!p) return false;
  size_t n = 0;
  while (p[n] && p[n] != ',' && p[n] != '*' && n < out_len - 1) {
    out[n] = p[n];
    n++;
  }
  out[n] = '\0';
  return n > 0;
}

}  // namespace

// --- framing ------------------------------------------------------------------

void NmeaParser::ProcessBytes(const uint8_t* buffer, size_t len) {
  while (len > 0) {
    // Find the first start-of-frame symbol: '$' (standard NMEA) or '#' (Unicore).
    if (line_len_ == 0) {
      const uint8_t* dollar = static_cast<const uint8_t*>(memchr(buffer, '$', len));
      const uint8_t* hash = static_cast<const uint8_t*>(memchr(buffer, '#', len));
      const uint8_t* start = dollar == nullptr ? hash : hash == nullptr ? dollar : (dollar < hash ? dollar : hash);
      if (start != nullptr) {
        len -= start - buffer;
        buffer = start;
      } else {
        return;
      }
    }

    const uint8_t* newline = static_cast<const uint8_t*>(memchr(buffer, '\n', len));
    if (newline != nullptr) {
      size_t bytes_to_take = newline - buffer + 1;
      if (line_len_ + bytes_to_take + 1 <= sizeof(line_)) {
        memcpy(&line_[line_len_], buffer, bytes_to_take);
        line_len_ += bytes_to_take;
        line_[line_len_] = '\0';
        ProcessLine(line_);
      }
      // else: line too long, drop it.
      len -= bytes_to_take;
      buffer = newline + 1;
      line_len_ = 0;
    } else {
      if (line_len_ + len + 2 <= sizeof(line_)) {
        memcpy(&line_[line_len_], buffer, len);
        line_len_ += len;
        return;
      }
      // Overflow without newline: drop partial line.
      line_len_ = 0;
      return;
    }
  }
}

void NmeaParser::Reset() {
  line_len_ = 0;
  gsv_fill_ = 0;
  gsa_used_fill_ = 0;
}

// --- sentence dispatch --------------------------------------------------------

bool NmeaParser::ProcessLine(const char* line) {
  if (line[0] == '#') {
    return ProcessUnicoreLine(line);
  }
  switch (minmea_sentence_id(line, true)) {
    case MINMEA_SENTENCE_GGA: {
      struct minmea_sentence_gga gga;
      if (!minmea_parse_gga(&gga, line)) {
        return false;
      }

      state_.lat = minmea_tocoord(&gga.latitude);
      state_.lon = minmea_tocoord(&gga.longitude);
      state_.height = minmea_tofloat(&gga.altitude);

      // GGA quality is the authoritative, per-epoch source for fix_type and
      // rtk_type — set unconditionally every epoch so a drop from RTK to single
      // can never latch a stale "fixed" value (the original firmware's #1
      // correctness concern). fix_type uses the GnssDetail scale and matches
      // exactly what the old ROS GpsServiceInterface published: RTK fixed = 5,
      // RTK float = 4, no fix = 0, any other valid fix (single/DGPS/PPS/DR) = 2.
      // solution_status keeps the finer ladder (0 none,1 single,2 DGPS,3 float,
      // 4 fixed) for receivers without Unicore #PVTSLN.
      switch (gga.fix_quality) {
        case 4:  // RTK fixed
          state_.rtk_type = 2;
          state_.fix_type = 5;
          state_.solution_status = 4;
          break;
        case 5:  // RTK float
          state_.rtk_type = 1;
          state_.fix_type = 4;
          state_.solution_status = 3;
          break;
        case 2:  // DGPS/differential
        case 3:  // PPS -> differential
          state_.rtk_type = 0;
          state_.fix_type = 2;
          state_.solution_status = 2;
          break;
        case 1:  // single point
        case 6:  // dead reckoning -> single
          state_.rtk_type = 0;
          state_.fix_type = 2;
          state_.solution_status = 1;
          break;
        default:  // 0 / invalid -> no fix
          state_.rtk_type = 0;
          state_.fix_type = 0;
          state_.solution_status = 0;
          break;
      }
      fix_quality_ = gga.fix_quality;

      state_.sats_used = gga.satellites_tracked;

      // Age of differential corrections (GGA field 13); scale 0 means empty.
      state_.correction_age = gga.dgps_age.scale != 0 ? static_cast<float>(minmea_tofloat(&gga.dgps_age)) : 0.0f;

      // GGA is the epoch boundary: publish the accumulated satellites + detail.
      CommitGsv();
      EmitEpoch(state_);
      return true;
    }

    case MINMEA_SENTENCE_RMC: {
      struct minmea_sentence_rmc rmc;
      if (!minmea_parse_rmc(&rmc, line)) {
        return false;
      }
      state_.lat = minmea_tocoord(&rmc.latitude);
      state_.lon = minmea_tocoord(&rmc.longitude);

      double speed = minmea_tofloat(&rmc.speed) * 0.514444;  // knots -> m/s
      double angle_rad = minmea_tofloat(&rmc.course) * M_PI / 180.0;
      state_.vel_e = sin(angle_rad) * speed;
      state_.vel_n = cos(angle_rad) * speed;
      state_.vel_u = 0;

      if (speed > 0.1) {
        double course_deg = minmea_tofloat(&rmc.course);
        double motion_heading = -course_deg * (M_PI / 180.0) + M_PI_2;
        motion_heading = fmod(motion_heading, 2.0 * M_PI);
        while (motion_heading < 0) motion_heading += 2.0 * M_PI;
        state_.motion_heading = motion_heading;
      }
      return true;
    }

    case MINMEA_SENTENCE_GSA: {
      struct minmea_sentence_gsa gsa;
      if (!minmea_parse_gsa(&gsa, line)) {
        return false;
      }
      // GGA owns fix_type/rtk_type authoritatively (above); GSA contributes only
      // the DOP breakdown and the used-in-solution PRN list.
      if (gsa.pdop.scale != 0) state_.pdop = static_cast<float>(minmea_tofloat(&gsa.pdop));
      if (gsa.hdop.scale != 0) state_.hdop = static_cast<float>(minmea_tofloat(&gsa.hdop));
      if (gsa.vdop.scale != 0) state_.vdop = static_cast<float>(minmea_tofloat(&gsa.vdop));
      AccumulateGsaUsed(line, gsa.sats);
      return true;
    }

    case MINMEA_SENTENCE_GSV: {
      ProcessGsv(line);
      return true;
    }

    case MINMEA_SENTENCE_GST: {
      struct minmea_sentence_gst gst;
      if (!minmea_parse_gst(&gst, line)) {
        return false;
      }
      float lat_std = minmea_tofloat(&gst.latitude_error_deviation);
      float lon_std = minmea_tofloat(&gst.longitude_error_deviation);
      float alt_std = minmea_tofloat(&gst.altitude_error_deviation);
      state_.h_acc = sqrt(lat_std * lat_std + lon_std * lon_std);
      state_.v_acc = alt_std;
      return true;
    }

    case MINMEA_INVALID: return false;

    default: ParseHDT(line); return true;
  }
}

void NmeaParser::ProcessGsv(const char* line) {
  struct minmea_sentence_gsv gsv;
  if (!minmea_parse_gsv(&gsv, line)) {
    return;
  }

  const uint8_t gnss_id = GnssIdFromTalker(line);
  const uint8_t sig_id = SignalIdFromGsv(line);
  const uint8_t band = BandFromSignal(gnss_id, sig_id);

  for (int i = 0; i < 4 && gsv_fill_ < kMaxSats; i++) {
    const struct minmea_sat_info& sat = gsv.sats[i];
    if (sat.nr == 0) {
      continue;
    }
    SatInfo& out = gsv_scratch_[gsv_fill_++];
    out.gnss_id = gnss_id;
    out.sv_id = static_cast<uint8_t>(sat.nr);
    out.cn0 = sat.snr < 0 ? 0 : static_cast<uint8_t>(sat.snr);
    out.band = band;
    if (sat.elevation == 0 && sat.azimuth == 0) {
      out.elevation = -128;
      out.azimuth = -1;
    } else {
      out.elevation = static_cast<int16_t>(sat.elevation);
      out.azimuth = static_cast<int16_t>(sat.azimuth);
    }
    out.used = false;
    out.healthy = sat.snr > 0;
  }
}

void NmeaParser::AccumulateGsaUsed(const char* line, const int* sats) {
  const char* star = strchr(line, '*');
  if (star == nullptr) return;
  const char* p = star;
  while (p > line && *(p - 1) != ',') p--;
  int system_id = (p < star) ? atoi(p) : 0;
  const uint8_t gnss_id = GnssIdFromSystemId(system_id);
  if (gnss_id == GNSS_UNKNOWN) return;
  for (int i = 0; i < 12 && gsa_used_fill_ < kMaxSats; i++) {
    if (sats[i] == 0) continue;
    gsa_used_[gsa_used_fill_].gnss_id = gnss_id;
    gsa_used_[gsa_used_fill_].sv_id = static_cast<uint8_t>(sats[i]);
    gsa_used_fill_++;
  }
}

void NmeaParser::CommitGsv() {
  // Mirror the firmware: only overwrite the published satellite list when this
  // epoch actually carried GSV data; otherwise keep the last sky so the panels
  // do not blank for a GGA that arrived before any GSV.
  if (gsv_fill_ == 0) {
    gsa_used_fill_ = 0;
    return;
  }
  // Mark used-in-solution from the GSA PRN list.
  for (uint8_t i = 0; i < gsv_fill_; i++) {
    gsv_scratch_[i].used = false;
    for (uint8_t j = 0; j < gsa_used_fill_; j++) {
      if (gsa_used_[j].gnss_id == gsv_scratch_[i].gnss_id && gsa_used_[j].sv_id == gsv_scratch_[i].sv_id) {
        gsv_scratch_[i].used = true;
        break;
      }
    }
  }
  const uint8_t n = gsv_fill_ < kMaxSats ? gsv_fill_ : kMaxSats;
  state_.satellites.assign(gsv_scratch_.begin(), gsv_scratch_.begin() + n);
  state_.sats_visible = n;
  gsv_fill_ = 0;
  gsa_used_fill_ = 0;
}

bool NmeaParser::ParseHDT(const char* line) {
  char type[6] = {};
  struct minmea_float heading = {};
  char t_indicator = 0;

  if (!minmea_scan(line, "tfc", type, &heading, &t_indicator)) {
    return false;
  }
  if (strncmp(type + 2, "HDT", 3) != 0) {
    return false;
  }
  if (t_indicator != 'T') {
    return false;
  }
  if (heading.scale == 0) {
    return false;
  }
  double heading_deg = minmea_tofloat(&heading);
  double heading_rad = -heading_deg * (M_PI / 180.0) + M_PI_2;
  heading_rad = fmod(heading_rad, 2.0 * M_PI);
  while (heading_rad < 0) heading_rad += 2.0 * M_PI;
  state_.vehicle_heading = heading_rad;
  // HDT carries no accuracy; use the firmware's small non-zero sentinel (~0.57 deg).
  state_.heading_accuracy = 0.57f;
  return true;
}

bool NmeaParser::ProcessUnicoreLine(const char* line) {
  const char* body = strchr(line, ';');
  body = body ? body + 1 : line;
  char f[24];
  constexpr size_t kFieldLen = sizeof(f);

  // #PVTSLNA: body[0]=position type, [7]=diff age, [21]=heading baseline,
  // [33]=elevation cutoff.
  if (strncmp(line, "#PVTSLN", 7) == 0) {
    if (UnicoreField(body, 0, f, kFieldLen)) {
      const uint8_t sol = UnicoreSolutionStatus(f);
      if (sol != 255) state_.solution_status = sol;
    }
    if (UnicoreField(body, 7, f, kFieldLen)) state_.correction_age = static_cast<float>(atof(f));
    if (UnicoreField(body, 21, f, kFieldLen)) state_.baseline_len = static_cast<float>(atof(f));
    if (UnicoreField(body, 33, f, kFieldLen)) state_.elevation_cutoff = static_cast<float>(atof(f));
    return true;
  }

  // #UNIHEADINGA (not #UNIHEADING2): the only NMEA-mode source of heading stddev.
  if (strncmp(line, "#UNIHEADING", 11) == 0 && strncmp(line, "#UNIHEADING2", 12) != 0) {
    char sol_stat[24] = {};
    char pos_type[24] = {};
    UnicoreField(body, 0, sol_stat, sizeof(sol_stat));
    UnicoreField(body, 1, pos_type, sizeof(pos_type));
    const bool computed =
        strcmp(sol_stat, "SOL_COMPUTED") == 0 && strstr(pos_type, "INS") == nullptr && strcmp(pos_type, "NONE") != 0;
    if (UnicoreField(body, 2, f, kFieldLen)) state_.baseline_len = static_cast<float>(atof(f));
    if (computed && UnicoreField(body, 3, f, kFieldLen)) {
      double heading_deg = atof(f);
      double heading_rad = -heading_deg * (M_PI / 180.0) + M_PI_2;
      heading_rad = fmod(heading_rad, 2.0 * M_PI);
      while (heading_rad < 0) heading_rad += 2.0 * M_PI;
      state_.vehicle_heading = heading_rad;
      if (UnicoreField(body, 6, f, kFieldLen)) {
        state_.heading_accuracy = static_cast<float>(atof(f));  // deg stddev
      }
    }
    return true;
  }

  // #AGCA: per-antenna AGC. body[0..4]=ANT1 bands, [5..9]=ANT2 bands; -1 = unused.
  if (strncmp(line, "#AGC", 4) == 0) {
    state_.antenna_agc.assign(10, -1);
    bool any = false;
    for (int i = 0; i < 10; i++) {
      if (UnicoreField(body, i, f, kFieldLen)) {
        state_.antenna_agc[i] = static_cast<int8_t>(atoi(f));
        any = true;
      }
    }
    if (!any) state_.antenna_agc.clear();
    return true;
  }

  // #JAMSTATUSA: body[1]=CWRatio (0..255), [2]=CWFlag (0/1/2).
  if (strncmp(line, "#JAMSTATUS", 10) == 0) {
    state_.jamming.assign(2, 0);
    if (UnicoreField(body, 1, f, kFieldLen)) state_.jamming[0] = static_cast<uint8_t>(atoi(f));
    if (UnicoreField(body, 2, f, kFieldLen)) state_.jamming[1] = static_cast<uint8_t>(atoi(f));
    return true;
  }

  return true;  // Unknown Unicore frame — ignore.
}

std::vector<std::vector<uint8_t>> NmeaParser::StartupCommands() const {
  // RAM-level LOG/CONFIG directives (no MODE/SAVECONFIG) requesting the detail
  // sentences this parser consumes. A non-Unicore NMEA receiver ignores the
  // proprietary ones. Mirrors the directives the firmware used to send.
  static const char* const kCommands[] = {
      "CONFIG NMEA0183 V411\r\n",     "LOG GPGGA ONTIME 1\r\n", "LOG GPGSV ONTIME 1\r\n",
      "LOG GLGSV ONTIME 1\r\n",       "LOG GAGSV ONTIME 1\r\n", "LOG GBGSV ONTIME 1\r\n",
      "LOG GPGSA ONTIME 1\r\n",       "LOG GPGST ONTIME 1\r\n", "LOG PVTSLNA ONTIME 1\r\n",
      "LOG UNIHEADINGA ONTIME 1\r\n", "LOG AGCA ONTIME 1\r\n",  "LOG JAMSTATUSA ONTIME 1\r\n",
  };
  std::vector<std::vector<uint8_t>> out;
  for (const char* cmd : kCommands) {
    out.emplace_back(cmd, cmd + strlen(cmd));
  }
  return out;
}

}  // namespace gnss_detail_parser
