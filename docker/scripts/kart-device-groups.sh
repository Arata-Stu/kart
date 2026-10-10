#!/usr/bin/env bash
# Root entrypoint extension: preserve Docker --group-add across gosu and docker exec.
# Runs after admin is created, before workspace-entrypoint.sh invokes gosu.
(
    set -euo pipefail
    : "${USERNAME:?workspace entrypoint must set USERNAME}"
    if [[ "$(id -u)" != 0 ]]; then
        echo 'kart-device-groups: must run in the root entrypoint' >&2
        exit 1
    fi
    kart_device_gids=$(id -G) || exit 1
    for kart_device_gid in $kart_device_gids; do
        [[ "$kart_device_gid" =~ ^[0-9]+$ ]] || exit 1
        # Do not grant the non-root user membership of root.
        [[ "$kart_device_gid" == 0 ]] && continue
        if ! getent group "$kart_device_gid" >/dev/null; then
            groupadd --gid "$kart_device_gid" "kart_device_${kart_device_gid}" || exit 1
        fi
        usermod --append --groups "$kart_device_gid" "$USERNAME" || exit 1
    done
    # The NVIDIA runtime can recreate Jetson GPU nodes as root:root 0600,
    # even when the host nodes are root:video 0660. Restore only these two
    # character devices before dropping privileges; never chmod all of /dev.
    for kart_gpu_device in /dev/nvhost-gpu /dev/nvhost-ctrl-gpu; do
        [[ -c "$kart_gpu_device" && ! -L "$kart_gpu_device" ]] || continue
        getent group video >/dev/null || { echo 'Missing video group' >&2; exit 1; }
        chgrp video "$kart_gpu_device" || exit 1
        chmod 0660 "$kart_gpu_device" || exit 1
        usermod --append --groups video "$USERNAME" || exit 1
        echo "kart GPU permissions: $kart_gpu_device -> video 0660"
    done
    echo "kart device groups registered: $(id -G "$USERNAME")"
) || exit 1
