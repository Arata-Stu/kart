#!/usr/bin/env bash
set -euo pipefail
KART_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Remember the explicitly selected profile in this checkout (never source it).
KART_PROFILE_FILE="${KART_ROOT}/.kart-dev-profile"
KART_DEV_PROFILE=standard
KART_PROFILE_EXPLICIT=false
if [[ -f "$KART_PROFILE_FILE" ]]; then
    KART_DEV_PROFILE=$(cat "$KART_PROFILE_FILE")
fi
case "${1:-}" in
    --evs) KART_DEV_PROFILE=evs; KART_PROFILE_EXPLICIT=true; shift ;;
    --no-evs) KART_DEV_PROFILE=standard; KART_PROFILE_EXPLICIT=true; shift ;;
esac
case "$KART_DEV_PROFILE" in
    standard|evs) ;;
    *) echo 'Invalid .kart-dev-profile; select --evs or --no-evs.' >&2; exit 1 ;;
esac
KART_EVS_ARGS=()
if [[ "$KART_DEV_PROFILE" == evs ]]; then
    # Existing images can run without retaining SDK build inputs on the host.
    for arg in "$@"; do
        if [[ "$arg" == --build || "$arg" == --build-local ]]; then
            for part in hal hal_psee_plugins licensing; do
                if [[ ! -d "${KART_ROOT}/docker/silky_evcam_plugin_source/${part}" ]] || [[ -z "$(find "${KART_ROOT}/docker/silky_evcam_plugin_source/${part}" -type f -print -quit)" ]]; then
                    echo "Missing SilkyEvCam source: docker/silky_evcam_plugin_source/${part}; see docs/setup/evs.md" >&2
                    exit 1
                fi
            done
        fi
    done
    KART_EVS_ARGS=(-c 'docker.image.additional_image_keys=[realsense,openeb,kart]' -c 'docker.run.container_name=kart_evs_dev')
fi
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
if [[ "$KART_PROFILE_EXPLICIT" == true && "${1:-}" != --help && "${1:-}" != -h ]]; then
    printf '%s\n' "$KART_DEV_PROFILE" > "$KART_PROFILE_FILE"
fi
if [[ "${1:-}" != --help && "${1:-}" != -h ]]; then
    echo "Docker profile: $KART_DEV_PROFILE (saved per checkout)"
fi
if [[ "$KART_DEV_PROFILE" == evs ]]; then
    isaac-ros activate "${KART_EVS_ARGS[@]}" "$@"
else
    isaac-ros activate "$@"
fi
