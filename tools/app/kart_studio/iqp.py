"""Bound the upstream IQP loop inside the single-use optimizer subprocess."""

import time


def bounded_iqp(tph, args, *, max_iterations=20, history=None):
    history = history if history is not None else []
    solve = tph.opt_min_curv.opt_min_curv
    iterations = []
    history.append(iterations)

    def step(**kwargs):
        if len(iterations) >= max_iterations:
            error = ValueError(
                f"反復最小曲率が{max_iterations}回で収束しませんでした。"
                f"最終線形化誤差={iterations[-1]['curvature_error']:.4g} 1/m"
                f"（収束条件={args['curv_error_allowed']:.4g} 1/m）。"
                "最小曲率（helper）を試すか、境界の急な折れ・点間隔・曲率上限を確認してください。"
            )
            error.diagnostic = dict(reason="iqp_nonconvergence", iterations=iterations)
            raise error
        count = len(kwargs["reftrack"])
        print(f"IQP iteration {len(iterations) + 1}/{max_iterations}: solving {count} points", flush=True)
        started = time.monotonic()
        result = solve(**kwargs)
        record = dict(iteration=len(iterations) + 1, point_count=count,
                      curvature_error=float(result[1]), elapsed_s=time.monotonic() - started)
        iterations.append(record)
        print(f"IQP iteration {record['iteration']}: error={record['curvature_error']:.6g} 1/m, "
              f"elapsed={record['elapsed_s']:.2f}s", flush=True)
        return result

    # Upstream has no iteration-limit argument. Restore its function even on
    # failure; this adapter runs only in a dedicated, single-use process.
    tph.opt_min_curv.opt_min_curv = step
    try:
        return tph.iqp_handler.iqp_handler(**args)
    finally:
        tph.opt_min_curv.opt_min_curv = solve
