# Joint observation in one Frenet reference. header.stamp is the oldest source
# measurement, NOT processing time. frame_id identifies the Cartesian map frame.
# s is unwrapped metres along reference_id, d is left positive. Both vehicles
# must use the SAME lap origin. A projection/tracking frontend must provide this.
std_msgs/Header header
string reference_id
uint64 track_id
float64 course_length
float64 ego_s
float64 ego_d
float64 ego_speed
float64 opponent_s
float64 opponent_d
float64 opponent_speed
float64 opponent_d_variance
float64 opponent_speed_variance
# false means unknown/lost, NEVER a clear track.
bool observed
# Conservative speed and minimum corridor widths for the WHOLE passing zone.
# passing_allowed only for a verified straight, obstacle-free corridor, with
# all-round opponent coverage. Current camera-only operation cannot assert this.
float64 speed_cap
float64 zone_end_s
float64 left_width
float64 right_width
bool passing_allowed
