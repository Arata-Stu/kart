"""Estimate visible tire/ground contact from a raw RGB box and a calibrated plane."""
import cv2
import numpy as np


def ground_point(uv, intrinsic, distortion, model, translation, quaternion, z, max_range):
    if model not in ("plumb_bob", "rational_polynomial"):
        return None
    k = np.asarray(intrinsic, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        return None
    d = np.asarray(distortion, dtype=float)
    if d.size not in (0, 4, 5, 8, 12, 14) or not np.isfinite(d).all():
        return None
    xy = cv2.undistortPoints(np.array(uv, dtype=float).reshape(1, 1, 2), k, d).ravel()
    ray = np.array([*xy, 1.0])
    q = np.array(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-8:
        return None
    q /= norm
    ray += 2 * np.cross(q[:3], np.cross(q[:3], ray) + q[3] * ray)
    origin = np.array(translation, dtype=float)
    if ray[2] >= -1e-6 or origin[2] <= z:
        return None
    point = origin + ray * ((z - origin[2]) / ray[2])
    if not np.isfinite(point).all() or np.linalg.norm(point[:2]) > max_range:
        return None
    return point
