#!/usr/bin/env bash
# Run in the EVS Docker container with GUI display forwarding.
set -euo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
kart_tuner_build="${kart_root}/tools/evs_bias_tuner/build"
if [[ "${1:-}" == --build ]]; then
    shift
    cmake -S "${kart_root}/tools/evs_bias_tuner" -B "$kart_tuner_build" \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local
    cmake --build "$kart_tuner_build" --parallel 2
fi
if [[ ! -x "${kart_tuner_build}/silkyevcam_bias_tuner" ]]; then
    echo '先に scripts/sensors/evs-bias.sh --build をEVSコンテナ内で実行してください。' >&2
    exit 1
fi
exec "${kart_tuner_build}/silkyevcam_bias_tuner" --output-dir "${kart_root}/bias/evs" "$@"
