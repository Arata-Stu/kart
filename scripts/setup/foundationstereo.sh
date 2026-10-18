#!/usr/bin/env bash
# Model setup runs on the target GPU, never during Docker build.
set -euo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
resolution=low_res
accept_eula=false
while (( $# )); do
    case "$1" in
        --model-res) resolution=${2:?--model-res requires low_res or high_res}; shift 2 ;;
        --eula) accept_eula=true; shift ;;
        -h|--help)
            echo 'Usage: bash scripts/setup/foundationstereo.sh [--model-res low_res|high_res] [--eula]'
            echo 'Existing non-empty engines are skipped. --eula accepts the NVIDIA model license.'
            exit 0 ;;
        *) echo "Unknown argument: $1" >&2; exit 2 ;;
    esac
done
case "$resolution" in
    low_res) size=320x736 ;;
    high_res) size=576x960 ;;
    *) echo 'model-res must be low_res or high_res' >&2; exit 2 ;;
esac
export ISAAC_ROS_WS=${ISAAC_ROS_WS:-$kart_root/ros2_ws}
engine="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationstereo/deployable_v2.0/foundationstereo_${size}.engine"
if [[ -f "$engine" && -s "$engine" && -r "$engine" ]]; then
    echo "準備済みのためスキップ: $engine"
    exit 0
fi
set +u
if [[ -f /opt/ros/lyrical/setup.bash ]]; then source /opt/ros/lyrical/setup.bash; fi
set -u
if ! command -v ros2 >/dev/null || ! ros2 pkg prefix isaac_ros_foundationstereo_models_install >/dev/null 2>&1; then
    echo '公式インストーラがありません。Docker imageを再buildするか、次を実行してください:' >&2
    echo 'sudo apt-get install -y ros-lyrical-isaac-ros-foundationstereo-models-install' >&2
    exit 1
fi
python3 - <<'PY'
import ctypes
cuda = ctypes.CDLL('libcuda.so.1')
code = cuda.cuInit(0)
if code:
    raise SystemExit(f'CUDA初期化に失敗: {code}。GPU公開設定・権限を確認してください。')
PY
if [[ "$accept_eula" != true ]]; then
    echo 'NVIDIA FoundationStereoモデルの利用条件を確認してください:'
    echo 'https://catalog.ngc.nvidia.com/orgs/nvidia/tao/models/foundationstereo'
    if [[ ! -t 0 ]]; then
        echo '非対話実行では、利用条件に同意した場合だけ --eula を指定してください。' >&2
        exit 1
    fi
    read -r -p '同意してモデル取得・engine生成を開始しますか？ [y/N] ' answer
    case "$answer" in y|Y|yes|YES) ;; *) echo '中止しました。'; exit 1 ;; esac
fi
echo "FoundationStereo setup: $resolution -> $engine"
ros2 run isaac_ros_foundationstereo_models_install install_foundationstereo_models.sh --eula --model_res "$resolution"
if [[ ! -f "$engine" || ! -s "$engine" || ! -r "$engine" ]]; then
    echo "engineが生成されませんでした: $engine" >&2
    exit 1
fi
echo "準備完了: $engine"
