"""Adapter for the pinned trajectory_planning_helpers fork; no global optimizer checkout."""

import json
import sys
from pathlib import Path


def run(document, p):
    from importlib.metadata import distribution

    import numpy as np
    import trajectory_planning_helpers as tph
    from scipy.interpolate import splev, splprep

    from .geometry import centerline

    center, _, corridor = centerline(
        document["left"], document["right"], True, p["spacing"]
    )
    ref = np.asarray(center)
    closed_ref = np.vstack((ref, ref[0]))
    spline, _ = splprep(closed_ref.T, s=len(ref) * 0.05**2, per=True)
    dense = np.asarray(splev(np.linspace(0, 1, max(1000, len(ref) * 10)), spline)).T
    stations = np.r_[0, np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))]
    target = np.linspace(0, stations[-1], len(ref), endpoint=False)
    ref = np.column_stack(
        [np.interp(target, stations, dense[:, axis]) for axis in range(2)]
    )
    corridor.validate(ref.tolist(), p["vehicle_width"] / 2 + p["margin"])
    cx, cy, matrix, normals = tph.calc_splines.calc_splines(
        path=np.vstack((ref, ref[0])), use_dist_scaling=False
    )

    # Measure actual available width along each spline normal, independent of
    # boundary labels/winding. Positive alpha follows the helper's right normal.
    def width(point, normal):
        distances = []
        for a, b in corridor.boundaries:
            segment = np.asarray(b) - a
            delta = np.asarray(a) - point
            det = normal[0] * segment[1] - normal[1] * segment[0]
            if abs(det) < 1e-10:
                continue
            t = (delta[0] * segment[1] - delta[1] * segment[0]) / det
            u = (delta[0] * normal[1] - delta[1] * normal[0]) / det
            if t > 0 and -1e-9 <= u <= 1 + 1e-9:
                distances.append(t)
        if not distances:
            raise ValueError("法線方向に境界が見つかりません")
        return min(distances)

    widths = np.asarray(
        [[width(pt, normal), width(pt, -normal)] for pt, normal in zip(ref, normals)]
    )
    track = np.column_stack((ref, widths))
    args = dict(
        reftrack=track,
        normvectors=normals,
        A=matrix,
        kappa_bound=p["curvature_limit"],
        w_veh=p["vehicle_width"] + 2 * (p["margin"] + 0.05),
        print_debug=False,
        plot_debug=False,
    )
    if p["optimizer"] == "mincurv":
        alpha = tph.opt_min_curv.opt_min_curv(**args)[0]
    else:
        lengths = tph.calc_spline_lengths.calc_spline_lengths(coeffs_x=cx, coeffs_y=cy)
        psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(
            coeffs_x=cx,
            coeffs_y=cy,
            ind_spls=np.arange(len(ref)),
            t_spls=np.zeros(len(ref)),
        )
        dkappa = (np.roll(kappa, -1) - np.roll(kappa, 1)) / (
            lengths + np.roll(lengths, 1)
        )
        result = tph.iqp_handler.iqp_handler(
            **args,
            spline_len=lengths,
            psi=psi,
            kappa=kappa,
            dkappa=dkappa,
            stepsize_interp=p["spacing"],
            iters_min=3,
            curv_error_allowed=0.01,
        )
        alpha, track, normals = result[:3]
    # A small interpolation interval also supports corridor checks on the output.
    result = tph.create_raceline.create_raceline(
        refline=track[:, :2],
        normvectors=normals,
        alpha=alpha,
        stepsize_interp=min(p["spacing"], 0.05),
    )[0]
    if not np.isfinite(result).all():
        raise ValueError("最適化結果に非有限値があります")
    package = distribution("trajectory-planning-helpers")
    return dict(
        points=result.tolist(),
        optimizer_info=dict(
            package="trajectory-planning-helpers",
            version=package.version,
            source=json.loads(package.read_text("direct_url.json") or "{}"),
            smoothing_rms_m=0.05,
            interpolation_reserve_m=0.05,
            verification_spacing_m=min(p["spacing"], 0.05),
        ),
    )


if __name__ == "__main__":
    data = json.loads(Path(sys.argv[1]).read_text())
    Path(sys.argv[2]).write_text(
        json.dumps(run(data["document"], data["parameters"]), allow_nan=False)
    )
