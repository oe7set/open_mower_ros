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
