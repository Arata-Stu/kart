"""Sample destination payload size without depending on scp's terminal meter."""

import subprocess
import threading
import time


def run_transfer(job, command, total, received):
    stop = threading.Event()
    started = time.monotonic()

    def report(count, phase):
        elapsed = max(time.monotonic() - started, 0.001)
        count = min(total, max(0, count))
        percent = 100 * count / total if total else 100
        job.log(
            f"{phase}: {percent:.1f}% · {count / 1048576:.1f}/{total / 1048576:.1f} MiB · 平均 {count / elapsed / 1048576:.2f} MiB/s"
        )

    def monitor():
        while not stop.wait(2):
            try:
                count = received()
                if not stop.is_set():
                    report(count, "転送中")
            except (OSError, ValueError, TimeoutError, subprocess.TimeoutExpired):
                # Progress sampling must not replace transfer integrity validation.
                pass

    report(0, "転送開始")
    thread = threading.Thread(target=monitor, daemon=True)
    thread.start()
    try:
        job.run(command)
    finally:
        stop.set()
        thread.join(timeout=31)
    report(total, "転送済み・整合性確認中")
