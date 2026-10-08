#!/usr/bin/env bash
# Portable tests only: no ROS daemon, USB device, or vehicle access.
set -euo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
kart_build=$(mktemp -d "${TMPDIR:-/tmp}/kart-vehicle-test.XXXXXX")
trap 'rm -rf "$kart_build"' EXIT
cxx=${CXX:-c++}
common=(-std=c++17 -Wall -Wextra -Wpedantic -UNDEBUG
  -I"$kart_root/ros2_ws/src/kart_system/include"
  -I"$kart_root/ros2_ws/src/kart_vehicle/include")
"$cxx" "${common[@]}" "$kart_root/ros2_ws/src/kart_system/test/test_control_core.cpp" -o "$kart_build/control"
"$kart_build/control"
"$cxx" "${common[@]}" "$kart_root/ros2_ws/src/kart_system/test/test_joy_controls.cpp" -o "$kart_build/joy_controls"
"$kart_build/joy_controls"
"$cxx" "${common[@]}" -pthread "$kart_root/ros2_ws/src/kart_vehicle/test/test_bridge_core.cpp" \
  "$kart_root/ros2_ws/src/kart_vehicle/src/bridge_protocol.cpp" -o "$kart_build/bridge"
"$kart_build/bridge"
"$cxx" "${common[@]}" "$kart_root/ros2_ws/src/kart_vehicle/test/test_esc_state.cpp" -o "$kart_build/esc"
"$kart_build/esc"
# Optional: pass the independent kart_bridge_board checkout; compiled read-only.
if [[ $# -gt 0 ]]; then
  firmware="$1/firmware"
  "${CC:-cc}" -std=c11 -I"$firmware/tests/stubs" -I"$firmware/common/Inc" \
    -c "$firmware/common/Src/jpbb_protocol.c" -o "$kart_build/firmware.o"
  "$cxx" "${common[@]}" -I"$firmware/tests/stubs" -I"$firmware/common/Inc" \
    "$kart_root/ros2_ws/src/kart_vehicle/test/test_firmware_compat.cpp" \
    "$kart_root/ros2_ws/src/kart_vehicle/src/bridge_protocol.cpp" "$kart_build/firmware.o" -o "$kart_build/compat"
  "$kart_build/compat"
fi
