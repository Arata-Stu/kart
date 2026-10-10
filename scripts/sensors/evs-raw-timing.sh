#!/usr/bin/env bash
set -euo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
kart_raw_build="${kart_root}/tools/evs_raw_timing/build"
if [[ "${1:-}" == --build ]]; then
    shift
    cmake -S "${kart_root}/tools/evs_raw_timing" -B "$kart_raw_build" -DCMAKE_PREFIX_PATH=/usr/local
    cmake --build "$kart_raw_build" --parallel 2
fi
exec "${kart_raw_build}/evs_raw_timing" "$@"
