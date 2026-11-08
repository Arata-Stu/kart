"""Projection onto a closed Cartesian reference, in metres."""
import math
import numpy as np


class Course:
    def __init__(self, points, speeds):
        self.points = np.asarray(points, dtype=float)
        self.speeds = np.asarray(speeds, dtype=float)
        if (self.points.ndim != 2 or self.points.shape[1] != 2 or len(self.points) < 3 or
                self.speeds.shape != (len(self.points),) or not np.isfinite(self.points).all() or
                not np.isfinite(self.speeds).all() or np.min(self.speeds) < 0):
            raise ValueError("invalid reference")
        self.delta = np.roll(self.points, -1, axis=0) - self.points
        self.lengths = np.linalg.norm(self.delta, axis=1)
        if np.min(self.lengths) < 1e-5:
            raise ValueError("degenerate segment")
        self.unit = self.delta / self.lengths[:, None]
        self.starts = np.r_[0., np.cumsum(self.lengths[:-1])]
        self.length = float(self.lengths.sum())

    def unwrap(self, s, near):
        return near + (s - near + self.length / 2) % self.length - self.length / 2

    def project(self, xy):
        rel = np.asarray(xy) - self.points
        along = np.clip(np.sum(rel * self.unit, axis=1), 0., self.lengths)
        residual = rel - along[:, None] * self.unit
        distances = np.linalg.norm(residual, axis=1)
        i = int(distances.argmin())
        s = self.starts + along
        if (not np.isfinite(distances[i]) or distances[i] > 2 or
                np.any((distances < distances[i] + .20) & (np.abs(self.unwrap(s, s[i])-s[i]) > 1.0))):
            raise ValueError("outside reference or ambiguous projection")
        d = self.unit[i, 0]*residual[i, 1] - self.unit[i, 1]*residual[i, 0]
        return float(s[i]), float(d), float(self.speeds[i])


def velocity(samples, column, position_sigma):
    """Least-squares slope and variance over distinct image timestamps."""
    t = np.array([x[0]-samples[-1][0] for x in samples])
    y = np.array([x[column] for x in samples])
    t -= t.mean()
    y -= y.mean()
    norm = float(t @ t)
    slope = float(t @ y / norm)
    residual = y - t * slope
    variance = max(position_sigma**2, float(residual @ residual) / max(1, len(t)-2)) / norm
    if not math.isfinite(slope + variance):
        raise ValueError("invalid velocity")
    return slope, variance
