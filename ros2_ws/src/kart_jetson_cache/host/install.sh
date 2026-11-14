#!/usr/bin/env bash
# Explicit Jetson host setup. No operation is performed until a socket request.
set -euo pipefail
HOST_FILES="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$(uname -s)" != Linux || "$(uname -m)" != aarch64 || "$EUID" != 0 ]]; then
    echo 'Run on the Jetson host: sudo bash ros2_ws/src/kart_jetson_cache/host/install.sh' >&2
    exit 1
fi
if [[ -e /.dockerenv || -e /run/.containerenv ]]; then
    echo 'Run on the Jetson host, outside Docker/Podman.' >&2
    exit 1
fi
/usr/bin/python3 -I - "$HOST_FILES/cache_helper.py" <<'PY'
import runpy
import sys
runpy.run_path(sys.argv[1])['validate_host']()
PY
if ! getent group kart-cache >/dev/null; then
    groupadd --system kart-cache
fi
if [[ "$(getent group kart-cache | cut -d: -f3)" == 0 ]]; then
    echo 'kart-cache group must not have GID 0' >&2
    exit 1
fi
# Install copies, never execute a user-writable checkout from the root service.
install -d -o root -g root -m 0755 /usr/local/libexec/kart-cache-helper
install -o root -g root -m 0644 "$HOST_FILES/cache_helper.py" /usr/local/libexec/kart-cache-helper/cache_helper.py
install -o root -g root -m 0644 "$HOST_FILES/kart-cache-helper.service" /etc/systemd/system/kart-cache-helper.service
systemctl daemon-reload
systemctl enable --now kart-cache-helper.service
systemctl restart kart-cache-helper.service
KART_CACHE_GID="$(getent group kart-cache | cut -d: -f3)"
KART_CACHE_READY=false
for ((KART_CACHE_ATTEMPT=0; KART_CACHE_ATTEMPT<15; KART_CACHE_ATTEMPT++)); do
    if [[ -S /run/kart-cache/cache.sock ]] && \
       [[ "$(stat -c '%u:%g:%a' /run/kart-cache/cache.sock)" == "0:${KART_CACHE_GID}:660" ]]; then
        KART_CACHE_READY=true
        break
    fi
    sleep 0.2
done
if [[ "$KART_CACHE_READY" != true ]]; then
    echo 'Helper socket did not become ready; inspect: systemctl status kart-cache-helper.service' >&2
    exit 1
fi
echo 'Helper installed. Recreate the dev container with scripts/dev.sh to mount its socket and group.'
echo 'No cache was released by this installer; only explicit requests perform the operation.'
