# vehicle/trim/request: incremental adjustment in ROS normalized units.
# vehicle/trim/state: current accumulated adjustment (not including fixed board calibration).
std_msgs/Header header
float32 steering
float32 throttle
