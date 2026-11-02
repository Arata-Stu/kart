# Experimental shadow output. NOT an actuator command or ReferenceLine.
# Stamp retains oldest observation; validity is bounded from this stamp.
std_msgs/Header header
string reference_id
uint64 track_id
bool valid
float64 valid_for_sec
string state
string reason
float64 target_speed
float64 observation_coverage
float64 roc_start_s
float64 roc_end_s
# Spatial spline samples in the reference_id Frenet frame. These are NOT
# timestamped feasible vehicle predictions. Empty unless a pass plan exists.
float64[] stations
float64[] offsets
