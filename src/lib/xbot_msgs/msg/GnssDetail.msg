# Detailed GNSS receiver diagnostics for the u-center / UPrecise-style GNSS
# page. Complementary to GpsStatus (which carries only the aggregate fix
# quality). Published at the receiver's nav rate (typically ~1 Hz).

std_msgs/Header header

# Per-signal satellite detail (skyplot + signal-strength bars).
Satellite[] satellites

# Total satellites in view as reported by the receiver.
uint16 sats_visible
# Satellites used in the navigation solution.
uint16 sats_used

# Dilution of precision. 0 means "not reported".
float32 gdop
float32 pdop
float32 hdop
float32 vdop
float32 tdop

# Fix quality, same scale as GpsStatus.fix_type:
# 0 = no fix, 1 = 2D, 2 = 3D, 3 = DGPS/SBAS, 4 = RTK float, 5 = RTK fixed
uint8 fix_type
# 0 = none, 1 = RTK float, 2 = RTK fixed
uint8 rtk_type

# Absolute position echo so the page has one coherent snapshot.
float64 lat
float64 lon
float32 height
# Position accuracy in meters.
float32 h_acc
float32 v_acc

# Velocity in the ENU frame (m/s).
float32 vel_e
float32 vel_n
float32 vel_u

# Headings in radians.
float32 vehicle_heading
float32 motion_heading

# Age of the last applied RTCM/NTRIP correction in seconds (0 = none yet).
float32 correction_age

# Dual-antenna baseline length in meters (Unicore). 0 = not reported.
float32 baseline_len
# Heading accuracy / standard deviation in degrees (dual-antenna). 0 = not reported.
float32 heading_accuracy
# Refined solution status: 0 none, 1 single, 2 DGPS, 3 float, 4 fixed.
# 255 = not reported (fall back to fix_type/rtk_type).
uint8 solution_status

# Enforced elevation cutoff mask in degrees (Unicore PVTSLN). -1 = not reported.
float32 elevation_cutoff

# Per-antenna AGC (Unicore #AGC): ANT1 (master) = [0..4], ANT2 (slave) = [5..9],
# per frequency band; -1 = unused band. Empty = not reported.
int8[] antenna_agc
# Jamming detection (Unicore #JAMSTATUS): [cw_ratio (0..255), cw_flag (0/1/2)].
# Empty = not reported.
uint8[] jamming
