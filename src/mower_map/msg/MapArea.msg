string name
bool active
geometry_msgs/Polygon area
geometry_msgs/Polygon[] obstacles
float64 angle                # per-area override (radians); NaN means use the auto-detected polygon angle
int32 outline_count          # per-area override; -1 means use the global config value
int32 outline_overlap_count  # per-area override; -1 means use the global config value
float64 outline_offset       # per-area override; NaN means use the global config value
int32 fill_type              # per-area override (slic3r fill enum); -1 means use the global default_mow_pattern
float64 distance             # per-area tool width (m); NaN means use the global tool_width
float64 speed_mps            # per-area mowing speed (m/s); NaN means use the global FTC planner speed
