# A state to display to a human to tell what's going on
float32 battery_percentage
bool emergency
bool is_charging
bool rain_detected
float32 gps_percentage
float32 current_action_progress
string current_state                # Like "Cleaning"
string current_sub_state            # Like "doing the dishes"
int16 current_area
int16 current_path
int16 current_path_index
xbot_msgs/AbsolutePose robot_pose # Pose of the docking station

# Detailed GPS status (mirror of xbot_msgs/GpsStatus, populated by monitoring
# from the gps_status topic).
uint8 gps_fix_type
uint16 gps_satellite_count
float32 gps_pdop

# WLAN signal stats sampled from /proc/net/wireless. 0 means N/A (e.g. mower
# is on Ethernet only or the file is unavailable).
int16 wifi_signal_dbm
float32 wifi_link_quality