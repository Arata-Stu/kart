#!/usr/bin/env bash
# Explicit host setup; dev.sh never installs packages or restarts services.
set -euo pipefail
KART_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ "$(uname -s)" != Linux || "$(uname -m)" != aarch64 || "$EUID" != 0 ]]; then
    echo 'Run on the Jetson host: sudo ./scripts/setup/setup-jtop.sh "$(command -v uv)"' >&2
    exit 1
fi
UV_BIN="${1:?Pass the absolute path to the uv executable}"
[[ "$UV_BIN" == /* && -x "$UV_BIN" ]] || { echo 'Invalid uv executable' >&2; exit 1; }
source "${KART_ROOT}/docker/jetson-stats.env"
JTOP_BUILD_DIR="$(mktemp -d)"
trap 'rm -rf -- "$JTOP_BUILD_DIR"' EXIT
git init "$JTOP_BUILD_DIR"
git -C "$JTOP_BUILD_DIR" remote add origin https://github.com/rbonghi/jetson_stats.git
git -C "$JTOP_BUILD_DIR" fetch --depth 1 origin "$JETSON_STATS_REF"
git -C "$JTOP_BUILD_DIR" checkout --detach FETCH_HEAD
test "$(git -C "$JTOP_BUILD_DIR" rev-parse HEAD)" = "$JETSON_STATS_REF"
git -C "$JTOP_BUILD_DIR" apply "${KART_ROOT}/docker/patches/jetson-stats-7.2.0-library-probe.patch"
"$UV_BIN" pip install --python /usr/bin/python3 --break-system-packages \
    --reinstall-package jetson-stats "$JTOP_BUILD_DIR"
test "$(/usr/bin/python3 -c 'import jtop; print(jtop.__version__)')" = "$JETSON_STATS_VERSION"
/usr/bin/python3 -m jtop --install-service
systemctl restart jtop.service
