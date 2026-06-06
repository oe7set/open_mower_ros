#include "ubx_parser.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gnss_detail_parser {

// --- framing (ported from UbxGpsDriver::ProcessBytes) -------------------------

void UbxParser::ProcessBytes(const uint8_t* buffer, size_t len) {
  while (len > 0) {
    size_t consumed = ProcessRingBuffer(buffer, len);
    if (consumed == 0) break;
    buffer += consumed;
    len -= consumed;
  }
}

// Returns how many bytes of the input were consumed this call.
size_t UbxParser::ProcessRingBuffer(const uint8_t* buffer, size_t len) {
  const uint8_t* const start = buffer;
  while (len > 0) {
    if (!found_header_) {
      switch (gbuffer_fill_) {
        case 0: {
          const auto* header_start = static_cast<const uint8_t*>(memchr(buffer, 0xb5, len));
          if (header_start == nullptr) {
            return (buffer - start) + len;  // reject the whole input
          }
          len -= (header_start - buffer);
          buffer = header_start;
          gbuffer_[gbuffer_fill_++] = *buffer;
          buffer++;
          len--;
        } break;
        case 1:
          if (buffer[0] == 0x62) {
            gbuffer_[gbuffer_fill_++] = *buffer;
            found_header_ = true;
          } else {
            gbuffer_fill_ = 0;
          }
          buffer++;
          len--;
          break;
        default:
          gbuffer_fill_ = 0;
          buffer++;
          len--;
          break;
      }
      continue;
    }

    if (gbuffer_fill_ < 6) {
      size_t bytes_to_take = std::min(len, static_cast<size_t>(6 - gbuffer_fill_));
      memcpy(&gbuffer_[gbuffer_fill_], buffer, bytes_to_take);
      gbuffer_fill_ += bytes_to_take;
      buffer += bytes_to_take;
      len -= bytes_to_take;
      if (gbuffer_fill_ != 6) {
        return buffer - start;
      }
    } else {
      uint16_t payload_length = gbuffer_[5] << 8 | gbuffer_[4];
      uint16_t total_length = payload_length + 8;
      if (total_length > sizeof(gbuffer_)) {
        found_header_ = false;
        gbuffer_fill_ = 0;
        continue;
      }
      size_t bytes_to_take = std::min(len, static_cast<size_t>(total_length - gbuffer_fill_));
      memcpy(&gbuffer_[gbuffer_fill_], buffer, bytes_to_take);
      gbuffer_fill_ += bytes_to_take;
      buffer += bytes_to_take;
      len -= bytes_to_take;
      if (total_length > gbuffer_fill_) {
        return buffer - start;
      }
      if (ValidateChecksum(gbuffer_, total_length)) {
        ProcessUbxPacket(gbuffer_ + 2, gbuffer_fill_ - 4);
      }
      found_header_ = false;
      gbuffer_fill_ = 0;
    }
  }
  return buffer - start;
}

void UbxParser::Reset() {
  found_header_ = false;
  gbuffer_fill_ = 0;
  nav_sat_count_ = 0;
}

void UbxParser::CalculateChecksum(const uint8_t* packet, size_t size, uint8_t& ck_a, uint8_t& ck_b) {
  ck_a = 0;
  ck_b = 0;
  for (size_t i = 0; i < size; i++) {
    ck_a += packet[i];
    ck_b += ck_a;
  }
}

bool UbxParser::ValidateChecksum(const uint8_t* packet, size_t size) const {
  uint8_t ck_a, ck_b;
  CalculateChecksum(packet + 2, size - 4, ck_a, ck_b);
  return packet[size - 2] == ck_a && packet[size - 1] == ck_b;
}

void UbxParser::ProcessUbxPacket(const uint8_t* data, size_t size) {
  uint16_t packet_id = data[0] << 8 | data[1];
  const uint8_t* payload = data + 4;
  const size_t payload_size = size - 4;

  switch (packet_id) {
    case (UbxNavPvt::CLASS_ID << 8 | UbxNavPvt::MESSAGE_ID):
      if (payload_size == sizeof(UbxNavPvt)) {
        HandleNavPvt(reinterpret_cast<const UbxNavPvt*>(payload));
      }
      break;
    case (UbxNavSat::CLASS_ID << 8 | UbxNavSat::MESSAGE_ID): HandleNavSat(payload, payload_size); break;
    case (UbxNavSig::CLASS_ID << 8 | UbxNavSig::MESSAGE_ID): HandleNavSig(payload, payload_size); break;
    case (UbxNavDop::CLASS_ID << 8 | UbxNavDop::MESSAGE_ID):
      if (payload_size == sizeof(UbxNavDop)) {
        HandleNavDop(reinterpret_cast<const UbxNavDop*>(payload));
      }
      break;
    default: break;
  }
}

void UbxParser::HandleNavPvt(const UbxNavPvt* msg) {
  bool gnssFixOK = (msg->flags & 0b0000001);
  if (!gnssFixOK) {
    return;
  }
  if (msg->flags3 & 0b1) {  // invalid lat/lon/height
    return;
  }

  // fix_type uses the GnssDetail scale and matches what the old ROS
  // GpsServiceInterface published: RTK fixed = 5, RTK float = 4, no fix = 0,
  // any other valid fix (2D/3D/GNSS+DR single) = 2. Set every epoch.
  switch (msg->fixType) {
    case 2:  // 2D
    case 3:  // 3D
    case 4:  // GNSS + dead reckoning
      state_.fix_type = 2;
      break;
    default: state_.fix_type = 0; break;
  }

  bool diffSoln = (msg->flags & 0b0000010) >> 1;
  auto carrSoln = static_cast<uint8_t>((msg->flags & 0b11000000) >> 6);
  if (diffSoln && carrSoln == 1) {
    state_.rtk_type = 1;
    state_.fix_type = 4;  // RTK float
  } else if (diffSoln && carrSoln == 2) {
    state_.rtk_type = 2;
    state_.fix_type = 5;  // RTK fixed
  } else {
    state_.rtk_type = 0;
  }

  state_.lat = static_cast<double>(msg->lat) / 10000000.0;
  state_.lon = static_cast<double>(msg->lon) / 10000000.0;
  state_.height = static_cast<float>(msg->hMSL) / 1000.0f;
  state_.h_acc = static_cast<float>(msg->hAcc) / 1000.0f;
  state_.v_acc = static_cast<float>(msg->vAcc) / 1000.0f;

  state_.vel_e = msg->velE / 1000.0f;
  state_.vel_n = msg->velN / 1000.0f;
  state_.vel_u = -msg->velD / 1000.0f;

  state_.sats_used = msg->numSV;
  state_.pdop = msg->pDOP / 100.0f;

  double headMotion = msg->headMot / 100000.0;
  headMotion = -headMotion * (M_PI / 180.0);
  headMotion = fmod(headMotion + M_PI_2, 2.0 * M_PI);
  while (headMotion < 0) headMotion += M_PI * 2.0;
  state_.motion_heading = headMotion;

  double hedVeh = msg->headVeh / 100000.0;
  hedVeh = -hedVeh * (M_PI / 180.0);
  hedVeh = fmod(hedVeh + M_PI_2, 2.0 * M_PI);
  while (hedVeh < 0) hedVeh += M_PI * 2.0;
  state_.vehicle_heading = hedVeh;
  double headAcc = (msg->headAcc / 100000.0) * (M_PI / 180.0);
  state_.heading_accuracy = static_cast<float>(headAcc * 180.0 / M_PI);  // degrees

  // NAV-PVT is the epoch boundary.
  EmitEpoch(state_);
}

void UbxParser::HandleNavSat(const uint8_t* payload, size_t size) {
  if (size < sizeof(UbxNavSat)) return;
  const auto* header = reinterpret_cast<const UbxNavSat*>(payload);
  const size_t expected = sizeof(UbxNavSat) + static_cast<size_t>(header->numSvs) * sizeof(UbxNavSatSv);
  if (size != expected) return;

  const auto* sv = reinterpret_cast<const UbxNavSatSv*>(payload + sizeof(UbxNavSat));
  nav_sat_count_ = 0;
  for (uint8_t i = 0; i < header->numSvs && nav_sat_count_ < kMaxSats; i++) {
    nav_sat_[nav_sat_count_++] = sv[i];
  }
  RebuildSatelliteState();
}

void UbxParser::HandleNavSig(const uint8_t* payload, size_t size) {
  if (size < sizeof(UbxNavSig)) return;
  const auto* header = reinterpret_cast<const UbxNavSig*>(payload);
  const size_t expected = sizeof(UbxNavSig) + static_cast<size_t>(header->numSigs) * sizeof(UbxNavSigSig);
  if (size != expected) return;

  const auto* sig = reinterpret_cast<const UbxNavSigSig*>(payload + sizeof(UbxNavSig));
  std::vector<SatInfo> sats;
  sats.reserve(header->numSigs);
  for (uint8_t i = 0; i < header->numSigs && sats.size() < kMaxSats; i++) {
    const UbxNavSigSig& s = sig[i];
    SatInfo out;
    out.gnss_id = s.gnssId;
    out.sv_id = s.svId;
    out.cn0 = s.cno;
    out.band = BandFromSignal(s.gnssId, s.sigId);
    out.used = (s.sigFlags & UbxNavSig::SIGFLAGS_PR_USED) != 0;
    out.healthy = (s.sigFlags & UbxNavSig::SIGFLAGS_HEALTH_MASK) == UbxNavSig::SIGFLAGS_HEALTH_HEALTHY;
    out.elevation = -128;
    out.azimuth = -1;
    for (uint8_t j = 0; j < nav_sat_count_; j++) {
      if (nav_sat_[j].gnssId == s.gnssId && nav_sat_[j].svId == s.svId) {
        out.elevation = nav_sat_[j].elev;
        out.azimuth = nav_sat_[j].azim;
        break;
      }
    }
    sats.push_back(out);
  }
  // Match the original ROS behaviour: sats_visible reflects the count of
  // published per-signal rows (NAV-SIG), not the raw NAV-SAT satellite count.
  state_.sats_visible = sats.size();
  state_.satellites = std::move(sats);
}

void UbxParser::HandleNavDop(const UbxNavDop* msg) {
  state_.gdop = msg->gDOP / 100.0f;
  state_.pdop = msg->pDOP / 100.0f;
  state_.tdop = msg->tDOP / 100.0f;
  state_.vdop = msg->vDOP / 100.0f;
  state_.hdop = msg->hDOP / 100.0f;
}

void UbxParser::RebuildSatelliteState() {
  // NAV-SAT aggregate view (one C/N0 per satellite). NAV-SIG overwrites this
  // with true per-band rows when it arrives.
  std::vector<SatInfo> sats;
  sats.reserve(nav_sat_count_);
  for (uint8_t i = 0; i < nav_sat_count_ && sats.size() < kMaxSats; i++) {
    const UbxNavSatSv& sv = nav_sat_[i];
    SatInfo out;
    out.gnss_id = sv.gnssId;
    out.sv_id = sv.svId;
    out.cn0 = sv.cno;
    out.band = 0;
    out.elevation = sv.elev;
    out.azimuth = sv.azim;
    out.used = (sv.flags & UbxNavSat::FLAGS_SV_USED) != 0;
    out.healthy = (sv.flags & UbxNavSat::FLAGS_HEALTH_MASK) == UbxNavSat::FLAGS_HEALTH_HEALTHY;
    sats.push_back(out);
  }
  state_.sats_visible = sats.size();
  state_.satellites = std::move(sats);
}

std::vector<uint8_t> UbxParser::BuildValset(const uint32_t* keys, size_t num_keys) const {
  // UBX-CFG-VALSET (0x06 0x8A), RAM layer, enabling each key at 1/epoch.
  constexpr size_t kCfgHeader = 4;
  constexpr size_t kItemSize = 5;
  const size_t payload = kCfgHeader + num_keys * kItemSize;
  std::vector<uint8_t> frame(8 + payload, 0);
  frame[0] = 0xb5;
  frame[1] = 0x62;
  frame[2] = 0x06;
  frame[3] = 0x8a;
  frame[4] = static_cast<uint8_t>(payload & 0xff);
  frame[5] = static_cast<uint8_t>((payload >> 8) & 0xff);
  uint8_t* p = frame.data() + 6;
  *p++ = 0x00;  // version
  *p++ = 0x01;  // layer = RAM
  *p++ = 0x00;
  *p++ = 0x00;
  for (size_t i = 0; i < num_keys; i++) {
    *p++ = static_cast<uint8_t>(keys[i] & 0xff);
    *p++ = static_cast<uint8_t>((keys[i] >> 8) & 0xff);
    *p++ = static_cast<uint8_t>((keys[i] >> 16) & 0xff);
    *p++ = static_cast<uint8_t>((keys[i] >> 24) & 0xff);
    *p++ = 0x01;
  }
  uint8_t ck_a, ck_b;
  CalculateChecksum(frame.data() + 2, frame.size() - 4, ck_a, ck_b);
  frame[frame.size() - 2] = ck_a;
  frame[frame.size() - 1] = ck_b;
  return frame;
}

std::vector<std::vector<uint8_t>> UbxParser::StartupCommands() const {
  // Enable NAV-SAT, NAV-SIG and NAV-DOP at 1 Hz on every output port (RAM only).
  static constexpr uint32_t kKeys[] = {
      0x20910016, 0x20910017, 0x20910018, 0x20910019,  // NAV-SAT
      0x20910345, 0x20910346, 0x20910347, 0x20910348,  // NAV-SIG
      0x20910038, 0x20910039, 0x2091003a, 0x2091003b,  // NAV-DOP
  };
  std::vector<std::vector<uint8_t>> out;
  out.push_back(BuildValset(kKeys, sizeof(kKeys) / sizeof(kKeys[0])));
  return out;
}

}  // namespace gnss_detail_parser
