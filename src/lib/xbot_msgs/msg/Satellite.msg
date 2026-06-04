# A single tracked GNSS signal. One satellite may appear multiple times when
# it is tracked on more than one frequency band (L1 + L2 + L5).

# GNSS system identifier (u-blox convention):
# 0 = GPS, 1 = SBAS, 2 = Galileo, 3 = BeiDou, 5 = QZSS, 6 = GLONASS, 255 = unknown
uint8 gnss_id

# Satellite vehicle id within its constellation.
uint8 sv_id

# Carrier-to-noise density ratio in dB-Hz (0 = not tracked).
uint8 cn0

# Normalized frequency band: 1 = L1/E1/B1, 2 = L2/B2I, 5 = L5/E5/B2a, 0 = unknown.
uint8 band

# Elevation in degrees (-128 = unknown).
int16 elevation

# Azimuth in degrees, 0..360 (-1 = unknown).
int16 azimuth

# Bit flags: bit0 = used in navigation solution, bit1 = reported healthy.
uint8 flags
uint8 FLAG_USED=1
uint8 FLAG_HEALTHY=2
