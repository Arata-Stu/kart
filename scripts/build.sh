#!/usr/bin/env bash
# Run inside the kart ROS development container.
set -euo pipefail
kart_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
kart_profile=auto
kart_sim=auto
kart_jobs=''
while [[ $# -gt 0 ]]; do
    case "$1" in
        --profile)
            kart_profile=${2:?--profile requires auto, jetson or desktop}; shift 2 ;;
        --with-sim) kart_sim=include; shift ;;
        --without-sim) kart_sim=ignore; shift ;;
        --jobs) kart_jobs=${2:?--jobs requires a positive integer}; shift 2 ;;
        -h|--help)
            cat <<'HELP'
Usage: scripts/build.sh [wrapper options] [colcon build arguments]
  --profile auto|jetson|desktop  Default: auto (Linux ARM64 + Tegra/Jetson marker)
  --with-sim                   Do not automatically ignore kart_sim
  --without-sim                Ignore kart_sim on any host
  --jobs N                     Set per-package compiler jobs (MAKEFLAGS/CMake)
Put wrapper options before colcon arguments. Other arguments pass through unchanged.
Jetson defaults: ignore kart_sim, one package at a time, two compiler jobs.
Explicit kart_sim selection includes it unless --without-sim is requested.
Explicit --executor / --parallel-workers and existing build environment are respected.
Examples:
  scripts/build.sh
  scripts/build.sh --profile jetson --packages-up-to kart_bringup
  scripts/build.sh --with-sim --packages-up-to kart_sim kart_bringup
  scripts/build.sh --jobs 1 --packages-select kart_control
HELP
            exit 0 ;;
        *) break ;;
    esac
done
case "$kart_profile" in
    auto)
        kart_profile=desktop
        if [[ "$(uname -s)" == Linux && "$(uname -m)" == aarch64 ]]; then
            if [[ "$(uname -r)" == *tegra* || -f /etc/nv_tegra_release ]] ||
               { [[ -r /sys/firmware/devicetree/base/model ]] &&
                 LC_ALL=C tr '\000' '\n' < /sys/firmware/devicetree/base/model | LC_ALL=C grep -qi jetson; }; then
                kart_profile=jetson
            fi
        fi ;;
    jetson|desktop) ;;
    *) echo 'Invalid --profile: choose auto, jetson or desktop.' >&2; exit 2 ;;
esac
if [[ -n "$kart_jobs" && ! "$kart_jobs" =~ ^[1-9][0-9]*$ ]]; then
    echo '--jobs must be a positive integer.' >&2; exit 2
fi
# Inspect colcon options, without interpreting the trailing CMake arguments.
kart_explicit_sim=false
kart_selection=false
kart_executor=false
kart_ignore_option=false
for kart_arg in "$@"; do
    case "$kart_arg" in
        --cmake-args|--ament-cmake-args|--catkin-cmake-args) break ;;
        --executor|--executor=*|--parallel-workers|--parallel-workers=*) kart_executor=true; kart_selection=false ;;
        --packages-select|--packages-up-to|--packages-above|--packages-above-and-dependencies) kart_selection=true ;;
        --packages-select=kart_sim|--packages-up-to=kart_sim|--packages-above=kart_sim|--packages-above-and-dependencies=kart_sim) kart_explicit_sim=true ;;
        --packages-ignore|--packages-ignore=*) kart_ignore_option=true; kart_selection=false ;;
        --*) kart_selection=false ;;
        kart_sim) if [[ "$kart_selection" == true ]]; then kart_explicit_sim=true; fi ;;
    esac
done
if [[ "$kart_sim" == ignore && "$kart_explicit_sim" == true ]]; then
    echo '--without-sim conflicts with explicit kart_sim selection.' >&2; exit 2
fi
if [[ "$kart_sim" == auto ]]; then
    if [[ "$kart_profile" == jetson && "$kart_explicit_sim" == false ]]; then kart_sim=ignore
    else kart_sim=include; fi
fi
if ! command -v colcon >/dev/null 2>&1; then
    echo 'colconが見つかりません。./scripts/dev.shでROS開発コンテナへ入ってから実行してください。' >&2
    exit 1
fi
kart_command=(colcon build --symlink-install)
if [[ "$kart_profile" == jetson ]]; then
    if [[ "$kart_executor" == false ]]; then kart_command+=(--executor sequential); fi
    export CMAKE_BUILD_PARALLEL_LEVEL=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
    export MAKEFLAGS=${MAKEFLAGS-"-j${CMAKE_BUILD_PARALLEL_LEVEL} -l${CMAKE_BUILD_PARALLEL_LEVEL}"}
fi
if [[ -n "$kart_jobs" ]]; then
    export CMAKE_BUILD_PARALLEL_LEVEL=$kart_jobs
    export MAKEFLAGS="${MAKEFLAGS:-} -j${kart_jobs} -l${kart_jobs}"
fi
if [[ "$kart_sim" == ignore && "$kart_ignore_option" == false ]]; then
    # Ignore at discovery time, so unbuilt sim is absent from the dependency graph.
    kart_command+=(--packages-ignore kart_sim)
fi
kart_passthrough=false
for kart_arg in "$@"; do
    if [[ "$kart_passthrough" == false && "$kart_sim" == ignore ]]; then
        case "$kart_arg" in
            --cmake-args|--ament-cmake-args|--catkin-cmake-args) kart_passthrough=true ;;
            --packages-ignore) kart_command+=(--packages-ignore kart_sim); continue ;;
            --packages-ignore=*) kart_command+=(--packages-ignore kart_sim "${kart_arg#*=}"); continue ;;
        esac
    fi
    kart_command+=("$kart_arg")
done
printf 'Build profile: %s; kart_sim: %s; compiler jobs: %s; MAKEFLAGS: %s\n' \
    "$kart_profile" "$kart_sim" "${CMAKE_BUILD_PARALLEL_LEVEL:-colcon default}" "${MAKEFLAGS:-colcon default}"
printf 'Command:'; printf ' %q' "${kart_command[@]}"; printf '\n'
cd "$kart_root/ros2_ws"
exec "${kart_command[@]}"
