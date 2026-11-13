# Estimated visible ground-contact point for display and explicitly enabled behavior observation,
# in GroundDetections.header.frame_id [m].
# Not a 3-D box, object centre, persistent track, occupancy or control command.
string detection_id
# Original detector class ID (e.g. COCO car=2 or racing rc_car=0, obstacle=1).
string class_id
# Canonical meaning assigned by the selected model profile: rc_car or obstacle.
string class_name
float32 score
geometry_msgs/Point point
