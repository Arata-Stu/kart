# Valid results retain image capture time; watchdog clear events may use notification time.
# Points have already been transformed into this frame.
# Default consumers display these points; explicitly enabled live behavior may use them as observations.
# They do not establish 3-D object extents, occupancy or observed free space.
std_msgs/Header header
# True only for a healthy result (including zero detections/selected classes).
# False for a failure/timeout clear event; an invalid empty event is NOT zero detections.
bool valid
string reason
GroundDetection[] objects
