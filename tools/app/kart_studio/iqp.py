"""Monitor the upstream IQP loop inside the single-use optimizer subprocess."""

import time


def monitored_iqp(tph, args, *, history=None):
    history = history if history is not None else []
    solve = tph.opt_min_curv.opt_min_curv
    iterations = []
    history.append(iterations)

    def step(**kwargs):
        count = len(kwargs["reftrack"])
        print(f"IQP iteration {len(iterations) + 1}: solving {count} points", flush=True)
        started = time.monotonic()
        result = solve(**kwargs)
        record = dict(iteration=len(iterations) + 1, point_count=count,
                      curvature_error=float(result[1]), elapsed_s=time.monotonic() - started)
        iterations.append(record)
        print(f"IQP iteration {record['iteration']}: error={record['curvature_error']:.6g} 1/m, "
              f"elapsed={record['elapsed_s']:.2f}s", flush=True)
        return result

    # Keep upstream convergence criteria unchanged. Restore the function on
    # failure; this adapter runs only in a dedicated, single-use process.
    tph.opt_min_curv.opt_min_curv = step
    try:
        return tph.iqp_handler.iqp_handler(**args)
    finally:
        tph.opt_min_curv.opt_min_curv = solve
