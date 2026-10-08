#!/usr/bin/env python3
"""Read-only version/socket check; never change fan, clock or power settings."""
import pathlib
import platform
import sys


def main():
    if platform.system() != "Linux" or platform.machine() not in ("aarch64", "arm64"):
        return 0
    pin_file = pathlib.Path(__file__).resolve().parents[1] / "docker/jetson-stats.env"
    pins = dict(line.split("=", 1) for line in pin_file.read_text().splitlines()
                if line and not line.startswith("#"))
    expected = pins["JETSON_STATS_VERSION"]
    try:
        import jtop
        if jtop.__version__ != expected:
            raise RuntimeError(f"installed={jtop.__version__}, required={expected}")
        if not pathlib.Path("/run/jtop.sock").is_socket():
            raise RuntimeError("/run/jtop.sock is missing; check the host jtop service")
        # Upstream checks only major/minor. Inspect the pinned 7.2.0 handshake
        # to enforce the patch version too, even after a package-only update.
        class PinnedJtop(jtop.jtop):
            def _get_configuration(self):
                config = super()._get_configuration()
                if config.get("version") != expected:
                    raise RuntimeError(
                        f"server={config.get('version')}, required={expected}; restart the host service")
                return config

        with PinnedJtop() as monitor:
            if not monitor.ok():
                raise RuntimeError("jtop server did not return statistics")
    except Exception as exc:
        print(f"jtop check failed: {exc}\nSee README.md: jtop setup.", file=sys.stderr)
        return 1
    print(f"jtop client/server check passed ({expected})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
