"""Use the lane's published Centerline as the Raceline starting geometry."""

import math

from .geometry import Corridor, PathValidationError, centerline, edges, points, resample
from .storage import fingerprint


def validate(line, corridor, clearance, info):
    try:
        corridor.validate(line, clearance)
    except PathValidationError as exc:
        label = "保存済みCenterline" if info["source"] == "saved_centerline" else "境界中点"
        exc.diagnostic.update(stage="raceline_reference", reference_source=info["source"])
        exc.args = (f"Raceline初期経路（{label}）の検査: {exc}",)
        raise


def build(document, spacing):
    closed = document["closed"]
    saved = document.get("lines", {}).get("centerline")
    if saved is None:
        line, _, corridor = centerline(document["left"], document["right"], closed, spacing)
        info = dict(source="paired_boundaries", method="paired_boundary_midpoints")
    else:
        minimum = 3 if closed else 2
        line = points(saved["points"], minimum)
        corridor = Corridor(points(document["left"], minimum), points(document["right"], minimum), closed)
        info = dict(source="saved_centerline", method=saved.get("method", "unknown"),
                    source_points_hash=fingerprint(line))
        # Honor the requested optimization resolution, even if the saved Center
        # was generated at another spacing. Never edit its stored points.
        length = sum(math.dist(a, b) for a, b in edges(line, closed))
        count = max(12 if closed else 3, min(600, math.ceil(length / spacing)))
        line = resample(line, count, closed)
    validate(line, corridor, 0, info)
    return line, corridor, info
