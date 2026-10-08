# Static offline reference, point order is forward travel direction.
# Empty points invalidates the previous selection. No repeated endpoint for loops.
std_msgs/Header header
string lane_id
string line_type
bool closed
geometry_msgs/Point[] points
# One nonnegative speed in m/s per point. Zero is a stop target.
float64[] speeds
