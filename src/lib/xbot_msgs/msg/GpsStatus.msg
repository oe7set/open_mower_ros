# Detailed GPS receiver status, complementary to AbsolutePose. Frontend uses
# fix_type for the dashboard RTK chip and satellite_count + pdop for the
# heatmap correlation per sample.

std_msgs/Header header

# 0 = no fix
# 1 = 2D fix
# 2 = 3D fix
# 3 = DGPS / SBAS
# 4 = RTK float
# 5 = RTK fixed
uint8 fix_type

# Number of satellites used in the navigation solution.
uint16 satellite_count

# Position Dilution of Precision. Lower = better. Typical RTK fix values 0.5..1.5.
float32 pdop
