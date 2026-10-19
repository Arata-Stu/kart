#!/usr/bin/env python3
"""Compare OpenEB Python RAW buffers without modifying the recording."""
import argparse
import json
import sys


def summarize(events, previous=None):
    import numpy as np
    result = dict(count=len(events), dtype=str(events.dtype), itemsize=events.dtype.itemsize,
                  strides=list(events.strides))
    if not len(events):
        return result
    times = events['t'].astype(np.int64)
    backwards = np.flatnonzero(np.diff(times) < 0)
    result.update(first_us=int(times[0]), last_us=int(times[-1]), min_us=int(times.min()),
                  max_us=int(times.max()), negative_count=int((times < 0).sum()),
                  regressions=len(backwards), boundary_regression=previous is not None and int(times[0]) < previous)
    if len(backwards):
        index = int(backwards[0]) + 1
        result.update(first_regression_index=index,
                      nearby_times_us=times[max(0, index-3):index+4].tolist())
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('raw')
    p.add_argument('--batches', type=int, default=3)
    a = p.parse_args()
    if not 1 <= a.batches <= 100:
        p.error('batches must be 1..100')
    import numpy as np
    from metavision_core.event_io.raw_reader import RawReader, RawReaderBase
    print(json.dumps(dict(python=sys.executable, numpy=np.__version__, numpy_path=np.__file__)))
    for kind, options in ((RawReader, {}), (RawReaderBase, dict(ev_count=100000, delta_t=0))):
        try:
            with kind(a.raw, do_time_shifting=False, **options) as reader:
                previous = None
                for batch in range(a.batches):
                    if reader.is_done():
                        break
                    events = reader.load_n_events(100000).copy()
                    report = summarize(events, previous)
                    print(json.dumps(dict(reader=kind.__name__, batch=batch, **report)), flush=True)
                    previous = report.get('last_us', previous)
        except Exception as error:
            print(json.dumps(dict(reader=kind.__name__, error=str(error))), flush=True)


if __name__ == '__main__':
    main()
