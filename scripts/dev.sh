#!/usr/bin/env bash
set -euo pipefail
KART_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ISAAC_ROS_WS="${KART_ROOT}/ros2_ws"
export DOCKER_ARGS_FILE="${KART_ROOT}/docker/dockerargs"
if ! command -v isaac-ros >/dev/null 2>&1; then
    echo 'Install the kart CLI .deb first; see README.md.' >&2
    exit 1
fi
if [[ "${1:-}" != "--help" && "${1:-}" != "-h" && "$(uname -s)" == Linux && "$(uname -m)" == aarch64 ]]; then
    /usr/bin/python3 "${KART_ROOT}/scripts/check-jtop.py"
fi
# Build per-run arguments without writing host-specific GIDs into the repo.
if [[ "${1:-}" != --help && "${1:-}" != -h && "$(uname -s)" == Linux ]]; then
    KART_ARGS_FILE="$(mktemp "${TMPDIR:-/tmp}/kart-dockerargs.XXXXXX")"
    trap 'rm -f -- "$KART_ARGS_FILE"' EXIT
    /usr/bin/python3 "${KART_ROOT}/scripts/input-dockerargs.py" \
        "${KART_ROOT}/docker/dockerargs" "$KART_ARGS_FILE"
    export DOCKER_ARGS_FILE="$KART_ARGS_FILE"
fi
isaac-ros activate "$@"
