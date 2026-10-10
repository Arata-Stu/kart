#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAP=minicar_2026
MODE=ros
SENSOR_ARGS=()
while (($#)); do
  case "$1" in
    --map) MAP="${2:?--map requires a name}"; shift 2 ;;
    --sensors) SENSOR_ARGS+=(--sensors); shift ;;
    --stereo-hz|--rgb-hz|--imu-hz|--rig) SENSOR_ARGS+=("$1" "${2:?requires a value}"); shift 2 ;;
    --preview) MODE=preview; shift ;;
    --check) MODE=check; shift ;;
    --list) for path in "$ROOT"/maps/sim/*.json; do basename "$path" .json; done; exit 0 ;;
    --help|-h) echo 'Usage: scripts/sim.sh [--map NAME] [--list|--preview|--check] [--sensors] [--stereo-hz HZ] [--rgb-hz HZ] [--imu-hz HZ] [--rig JSON]'; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ "$MAP" != */* && -f "$ROOT/maps/sim/$MAP.json" ]] || { echo "Unknown map: $MAP" >&2; exit 2; }
if [[ "$MODE" == ros ]]; then
  [[ -z "${SENSOR_ARGS[*]-}" ]] || { echo "Sensor flags require --preview/--check; use ros2 launch arguments in ROS mode." >&2; exit 2; }
  exec ros2 launch kart_bringup sim.launch.py "map:=$MAP" "map_dir:=$ROOT/maps/sim"
fi
export PYTHONPATH="$ROOT/ros2_ws/src/kart_sim${PYTHONPATH:+:$PYTHONPATH}"
ARGS=(--map "$ROOT/maps/sim/$MAP.json")
# macOS Bash 3.2 treats an empty array as unset with nounset enabled.
[[ -z "${SENSOR_ARGS[*]-}" ]] || ARGS+=("${SENSOR_ARGS[@]}")
[[ "$MODE" != check ]] || ARGS+=(--check)
exec "${SIM_PYTHON:-python3}" -m kart_sim.preview "${ARGS[@]}"
