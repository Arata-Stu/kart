"""ROS-free, geometry-bound passing permissions shared by Studio and HDMap."""
import hashlib
import json
import math
import re

MAX_ZONES = 64
_FIELDS = {'id', 'start_index', 'end_index', 'side', 'geometry_revision'}


def _geometry(points, closed):
    if type(closed) is not bool:
        raise ValueError('Passing permissions require a closed boolean')
    if not isinstance(points, (list, tuple)) or not (3 if closed else 2) <= len(points) <= 1_000_000:
        raise ValueError('Passing permissions require a generated XY line')
    normalized = []
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError('Passing permissions require XY points in meters')
        row = []
        for value in point:
            if type(value) not in (int, float):
                raise ValueError('Passing line coordinates must be finite numbers')
            try:
                number = float(value)
            except (ValueError, OverflowError) as error:
                raise ValueError('Passing line coordinates must be finite numbers') from error
            if not math.isfinite(number):
                raise ValueError('Passing line coordinates must be finite numbers')
            row.append(0.0 if number == 0 else number)
        normalized.append(row)
    return dict(points=normalized, closed=closed)


def geometry_revision(points, closed):
    """Hash normalized XY vertex order and closure, independent of speed/profile.

    Integers and equal float values have the same representation; negative zero
    is zero. Vertices are never reordered, resampled or implicitly closed.
    """
    raw = json.dumps(_geometry(points, closed), sort_keys=True, separators=(',', ':'),
                     allow_nan=False).encode()
    return hashlib.sha256(raw).hexdigest()


def validate_passing_zones(zones, points, closed):
    """Return copied permissions, rejecting stale geometry and malformed ranges.

    Indices are zero-based inclusive endpoints. A closed line may wrap through
    index zero; an open line must increase. Missing permissions are represented
    by the caller as an empty list, never an implied whole-course permission.
    """
    if not isinstance(zones, list) or len(zones) > MAX_ZONES:
        raise ValueError('Passing permissions must be a list of at most 64 zones')
    revision = geometry_revision(points, closed)
    result, seen = [], set()
    for zone in zones:
        if not isinstance(zone, dict) or set(zone) != _FIELDS:
            raise ValueError('Passing zone needs id, start_index, end_index, side and geometry_revision')
        identity = zone['id']
        if (not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', identity)
                or identity.casefold() in seen):
            raise ValueError('Invalid or duplicate passing zone ID')
        start, end = zone['start_index'], zone['end_index']
        if (type(start) is not int or type(end) is not int or not 0 <= start < len(points)
                or not 0 <= end < len(points) or start == end or (not closed and start > end)):
            raise ValueError('Invalid passing zone waypoint range')
        if zone['side'] not in ('both', 'left', 'right'):
            raise ValueError('Passing zone side must be both, left or right')
        stored = zone['geometry_revision']
        if not isinstance(stored, str) or not re.fullmatch(r'[0-9a-f]{64}', stored):
            raise ValueError('Passing zone geometry_revision must be a SHA-256 digest')
        if stored != revision:
            raise ValueError('Passing zone geometry revision mismatch; re-authorize the changed line')
        seen.add(identity.casefold())
        result.append(dict(id=identity, start_index=start, end_index=end, side=zone['side'],
                           geometry_revision=stored))
    return result


def zone_vertices(points, zone):
    """Copy inclusive vertices from an already validated zone, including wraps."""
    start, end = zone['start_index'], zone['end_index']
    vertices = points[start:end + 1] if start < end else points[start:] + points[:end + 1]
    return [list(point) for point in vertices]
