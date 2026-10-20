# Visualization only: the point actually chosen by the controller.
# header uses the pose measurement stamp and map frame when valid.
std_msgs/Header header
bool valid
geometry_msgs/Point vehicle_position
geometry_msgs/Point target
# Configured forward arc length from the nearest path projection (m).
float64 lookahead_m
# Euclidean XY distance from vehicle_position to target (m).
float64 target_distance_m
