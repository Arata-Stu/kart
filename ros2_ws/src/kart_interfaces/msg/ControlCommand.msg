# Normalized actuator command, not speed/acceleration in physical units.
# steering: [-1,1], positive left. Others: [0,1].
# throttle, brake and reverse must not be positive simultaneously.
# Stamp is the original source time; mux must preserve it.
# All-zero means neutral, not an emergency brake.
std_msgs/Header header

float32 steering
float32 throttle
float32 brake
float32 reverse
