"""Speed/precision selection, independent of VO/VIO tracking_mode.

Isaac ROS pinned visual_slam_impl.cpp maps 0/1/2 to Moderate/Performance/Precision.
"""
LABELS={
    "0":"0 Moderate：速度と精度のバランス",
    "1":"1 Performance：速度重視・処理負荷を抑える",
    "2":"2 Precision：精度重視・処理負荷は増える",
}


def multicam_value(value):
    if str(value) not in LABELS:
        raise ValueError('multicam_mode must be 0 (Moderate), 1 (Performance), or 2 (Precision)')
    return int(value)
